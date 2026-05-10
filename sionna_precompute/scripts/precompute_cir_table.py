"""
Sionna RT grid sweep — generates CIR lookup table for all 5 anchors.

Usage:
    # Full-anchor mode (5 subprocesses, one per anchor — default):
    python scripts/precompute_cir_table.py [--dry-run] [--chunk-size N]

    # Half-anchor mode (10 subprocesses, two per anchor):
    python scripts/precompute_cir_table.py --half-anchors [--dry-run] [--chunk-size N]

Architecture — subprocess isolation + GPU tensor injection:
    Dr.JIT and Mitsuba maintain C++ process-level singletons that Python
    cleanup cannot reset. Each child subprocess gets a clean OS process,
    guaranteeing all C++ state is destroyed on exit.

    GPU target patch (option C):
        scene.targets() normally iterates over scene.receivers in Python,
        executing chunk_size × 3 dr.scatter() calls per PathSolver invocation.
        These accumulate in Dr.JIT's AD tape (9000 entries × 129 chunks =
        1.16 M entries), causing per-chunk time to grow exponentially.

        _install_gpu_target_patch() replaces scene.targets() with a version
        that builds a mi.Point3f directly from a numpy array in one bulk
        operation. No dr.scatter loop → tape stays flat → per-chunk time stays
        constant across all 129 chunks.

    Full-anchor (5 spawns): each child processes all ~387 chunks for one anchor.
    Half-anchor (10 spawns): each child processes ~65 chunks; two sequential
      children cover one anchor. Results merged in parent via temp .npz files.

    Both modes share identical child logic — the only difference is
    pt_start/pt_end bounding the flat-grid slice each child works on.
"""
import argparse
import datetime
import gc
import os
import subprocess
import sys
import time

import h5py
import numpy as np
import yaml

# ── repo paths ────────────────────────────────────────────────────────────────
SCRIPT_DIR     = os.path.dirname(os.path.abspath(__file__))
PRECOMPUTE_DIR = os.path.dirname(SCRIPT_DIR)
REPO_ROOT      = os.path.dirname(PRECOMPUTE_DIR)

SCENE_XML    = os.path.join(PRECOMPUTE_DIR, "scene", "room.xml")
ANCHORS_YAML = os.path.join(REPO_ROOT, "src", "microuwb_bringup", "config", "anchors.yaml")
DATA_DIR     = os.path.join(PRECOMPUTE_DIR, "data")

# ── solver constants ──────────────────────────────────────────────────────────
C        = 3e8
FREQ_HZ  = 6.5e9
MAX_DEPTH = 4
SAMPLES  = int(1e5)

GRID_X = (0.20, 4.80, 0.05)
GRID_Y = (0.20, 3.80, 0.05)
GRID_Z = (0.10, 2.90, 0.05)

DRY_X = (0.20, 4.80, 1.15)
DRY_Y = (0.20, 3.80, 0.90)
DRY_Z = (0.10, 2.90, 0.70)


def make_grid(x_params, y_params, z_params):
    xs = np.arange(x_params[0], x_params[1] + x_params[2] * 0.5, x_params[2])
    ys = np.arange(y_params[0], y_params[1] + y_params[2] * 0.5, y_params[2])
    zs = np.arange(z_params[0], z_params[1] + z_params[2] * 0.5, z_params[2])
    return xs, ys, zs


def load_anchors():
    with open(ANCHORS_YAML) as f:
        return yaml.safe_load(f)["anchors"]


def _trim_cuda():
    try:
        import drjit
        drjit.cuda.malloc_trim()
    except Exception:
        pass


def extract_metrics(paths, N_rx: int):
    """
    Vectorised extraction of per-receiver first-path metrics.

    paths.tau          : (N_rx, 1, n_paths)
    paths.valid        : same shape
    paths.interactions : (max_depth, N_rx, 1, n_paths)
    paths.a            : complex, same layout as tau

    Returns (range_m, los_flag, first_power_db, total_power_db), each (N_rx,).
    """
    NONE = InteractionType.NONE

    tau_np   = np.array(paths.tau)
    valid_np = np.array(paths.valid).astype(bool)
    if tau_np.ndim == 3:
        tau_np   = tau_np[:, 0, :]
        valid_np = valid_np[:, 0, :]

    inter_np = np.array(paths.interactions)
    if inter_np.ndim == 4:
        inter_np = inter_np[:, :, 0, :]

    try:
        a_raw    = np.array(paths.a)
        power_np = np.abs(a_raw) ** 2
        while power_np.ndim > 2 and power_np.shape[1] == 1:
            power_np = power_np[:, 0, ...]
        if power_np.ndim == 3 and power_np.shape[-1] == 1:
            power_np = power_np[..., 0]
        has_power = power_np.ndim == 2 and power_np.shape[0] == N_rx
    except Exception:
        has_power = False

    EPS = 1e-30

    range_m        = np.full(N_rx, np.nan, dtype=np.float32)
    los_flag       = np.zeros(N_rx, dtype=bool)
    first_power_db = np.full(N_rx, np.nan, dtype=np.float32)
    total_power_db = np.full(N_rx, np.nan, dtype=np.float32)

    tau_masked = np.where(valid_np, tau_np, np.inf)
    first_idx  = np.argmin(tau_masked, axis=1)
    has_any    = valid_np.any(axis=1)

    range_m[has_any] = tau_masked[has_any, first_idx[has_any]] * C

    inter_first   = inter_np[:, np.arange(N_rx), first_idx]
    los_flag[has_any] = np.all(inter_first[:, has_any] == NONE, axis=0)

    if has_power:
        power_masked = np.where(valid_np, power_np, 0.0)
        fp_linear    = power_masked[np.arange(N_rx), first_idx]
        tp_linear    = power_masked.sum(axis=1)
        fp_linear    = np.where(has_any, np.maximum(fp_linear, EPS), EPS)
        tp_linear    = np.where(has_any, np.maximum(tp_linear, EPS), EPS)
        first_power_db[has_any] = (10 * np.log10(fp_linear[has_any])).astype(np.float32)
        total_power_db[has_any] = (10 * np.log10(tp_linear[has_any])).astype(np.float32)

    return range_m, los_flag, first_power_db, total_power_db


def compute_variance(first_power_db: np.ndarray) -> np.ndarray:
    """
    0.001 m² (best LOS) → 0.050 m² (worst NLOS) → 0.250 m² (no path).
    Relative scale; absolute calibration in step-5 Kalman filter.
    """
    valid      = ~np.isnan(first_power_db)
    fp_linear  = np.where(valid, 10.0 ** (first_power_db / 10.0), 0.0)
    p_max      = fp_linear[valid].max() if valid.any() else 1.0
    power_norm = np.clip(fp_linear / max(p_max, 1e-30), 0.0, 1.0)
    variance   = np.where(valid, 0.001 + 0.049 * (1.0 - power_norm), 0.25)
    return variance.astype(np.float32)


# ── GPU target patch (option C) ───────────────────────────────────────────────

def _install_gpu_target_patch(scene, chunk_size: int) -> None:
    """
    Replace scene.targets() with a bulk GPU-tensor version.

    Original scene.targets() iterates over scene.receivers in Python and calls
    dr.scatter() for each one — O(chunk_size) tape entries per PathSolver call.
    Over 129 chunks that's 1.16 M tape entries causing exponential slowdown.

    Patched version builds mi.Point3f directly from scene._gpu_pts (a numpy
    float32 array set per-chunk), using one bulk mi.Float() constructor per
    axis — no scatter loop, no tape accumulation.
    """
    import types
    import mitsuba as mi
    import drjit as dr

    scene._gpu_pts = None   # (chunk_size, 3) float32, updated each chunk
    scene._gpu_n   = chunk_size

    def _patched_targets(self, synthetic_array, return_velocities):
        pts = self._gpu_pts          # float32 numpy, written by parent before each solver call
        n   = self._gpu_n
        # Single bulk host→GPU transfer per axis — no per-element Python ops
        pos = mi.Point3f(mi.Float(pts[:, 0]), mi.Float(pts[:, 1]), mi.Float(pts[:, 2]))
        ori = dr.zeros(mi.Point3f,  n)
        # rel_ant_positions: zero offset for single-element isotropic array
        rel = dr.zeros(mi.Point3f,  n)
        vel = dr.zeros(mi.Vector3f, n) if return_velocities else None
        return pos, ori, rel, vel

    scene.targets = types.MethodType(_patched_targets, scene)


# ── child-process entry point ─────────────────────────────────────────────────

def run_child_anchor(anc_idx: int, output_path: str,
                     xs, ys, zs, chunk_size: int, anchors: list,
                     pt_start: int = 0, pt_end: int = None):
    """
    Child subprocess: process flat-grid points [pt_start:pt_end] for anchor N.

    Full mode  (pt_end=None):  processes all points, writes anchor group to HDF5.
    Half mode  (pt_end set):   processes a slice, writes partial results to a
                               temp .npz file; parent merges after both halves.

    Uses GPU target patch: scene.targets() is replaced by a bulk mi.Point3f
    constructor so Dr.JIT tape entries stay O(1) per chunk rather than
    O(chunk_size). GC + CUDA trim every 5 chunks remains as safety valve.
    """
    import sionna.rt as rt
    from sionna.rt import PathSolver, InteractionType as _IT, Transmitter, Receiver

    global InteractionType
    InteractionType = _IT

    anchor    = anchors[anc_idx]
    anc_pos_i = [float(anchor["x"]), float(anchor["y"]), float(anchor["z"])]

    NX, NY, NZ = len(xs), len(ys), len(zs)
    grid_shape = (NX, NY, NZ)
    total_pts  = NX * NY * NZ

    if pt_end is None:
        pt_end = total_pts
    half_mode = (pt_start > 0 or pt_end < total_pts)
    n_local   = pt_end - pt_start
    n_chunks  = (n_local + chunk_size - 1) // chunk_size

    xi_idx, yi_idx, zi_idx = np.meshgrid(
        np.arange(NX), np.arange(NY), np.arange(NZ), indexing="ij")
    xi_flat  = xi_idx.ravel()
    yi_flat  = yi_idx.ravel()
    zi_flat  = zi_idx.ravel()
    pts_flat = np.stack([xs[xi_flat], ys[yi_flat], zs[zi_flat]], axis=1)

    # Slice to our portion of the flat grid
    our_xi   = xi_flat  [pt_start:pt_end]
    our_yi   = yi_flat  [pt_start:pt_end]
    our_zi   = zi_flat  [pt_start:pt_end]
    our_pts  = pts_flat [pt_start:pt_end]

    label = f"pts [{pt_start}:{pt_end}]" if half_mode else "all pts"
    print(f"\n[anchor_{anc_idx}] TX at {anc_pos_i} — "
          f"{n_local} pts ({label}) in {n_chunks} chunks of {chunk_size}", flush=True)

    # ── scene setup (once per child = once per fresh process) ─────────────────
    scene = rt.load_scene(SCENE_XML)
    scene.frequency = FREQ_HZ
    iso_array = rt.PlanarArray(num_rows=1, num_cols=1,
                               vertical_spacing=0.5, horizontal_spacing=0.5,
                               pattern="iso", polarization="V")
    scene.tx_array = iso_array
    scene.rx_array = iso_array
    scene.add(Transmitter(name="tx", position=anc_pos_i))

    # Register chunk_size receivers so Paths.__init__ sees num_rx=chunk_size.
    # scene.add() is pure Python dict insert (no Dr.JIT ops); adding 3000 takes ~ms.
    # Per-chunk scatter loop inside scene.targets() is bypassed by the GPU patch.
    pad_pos = our_pts[0].tolist()
    for k in range(chunk_size):
        scene.add(Receiver(name=f"rx_{k}", position=pad_pos))
    _install_gpu_target_patch(scene, chunk_size)

    solver = PathSolver()

    # ── result arrays (full-grid-shaped, NaN/False for unprocessed cells) ──────
    range_arr    = np.full(grid_shape, np.nan, dtype=np.float32)
    los_arr      = np.zeros(grid_shape, dtype=bool)
    fp_power_arr = np.full(grid_shape, np.nan, dtype=np.float32)
    tp_power_arr = np.full(grid_shape, np.nan, dtype=np.float32)

    t_anchor = time.time()
    t_prev   = t_anchor
    # float32: mi.Float() accepts float32 numpy directly; avoids a copy on cast.
    padded   = np.empty((chunk_size, 3), dtype=np.float32)

    for ci in range(n_chunks):
        local_start = ci * chunk_size
        local_end   = min(local_start + chunk_size, n_local)
        N_real      = local_end - local_start
        chunk       = our_pts[local_start:local_end]

        # Proactive GC + CUDA trim every 5 chunks as safety valve.
        if ci > 0 and ci % 5 == 0:
            gc.collect()
            _trim_cuda()

        padded[:N_real] = chunk
        if N_real < chunk_size:
            padded[N_real:] = chunk[0]
        # Bulk GPU tensor update — replaces the O(chunk_size) Python loop.
        scene._gpu_pts = padded   # patched scene.targets() reads this

        p = solver(scene, max_depth=MAX_DEPTH, los=True,
                   specular_reflection=True, diffuse_reflection=False,
                   refraction=False, diffraction=False,
                   samples_per_src=SAMPLES)

        r_chunk, l_chunk, fp_chunk, tp_chunk = extract_metrics(p, chunk_size)
        del p

        xi_c = our_xi[local_start:local_end]
        yi_c = our_yi[local_start:local_end]
        zi_c = our_zi[local_start:local_end]
        range_arr   [xi_c, yi_c, zi_c] = r_chunk[:N_real]
        los_arr     [xi_c, yi_c, zi_c] = l_chunk[:N_real]
        fp_power_arr[xi_c, yi_c, zi_c] = fp_chunk[:N_real]
        tp_power_arr[xi_c, yi_c, zi_c] = tp_chunk[:N_real]

        now     = time.time()
        t_last  = now - t_prev   # individual chunk wall time (not running avg)
        t_prev  = now
        elapsed = now - t_anchor
        done    = local_end / n_local
        eta     = elapsed / max(done, 1e-9) * (1 - done)
        t_avg   = elapsed / (ci + 1)

        # Log every chunk for the first 30, then every 10 — for flat-vs-growing comparison
        if ci < 30 or ci % 10 == 0 or ci == n_chunks - 1:
            print(f"  chunk {ci+1:4d}/{n_chunks}  {100*done:5.1f}%  "
                  f"avg {t_avg:.1f}s  last {t_last:.1f}s  "
                  f"elapsed {elapsed:.0f}s  ETA {eta:.0f}s", flush=True)

    anchor_time = time.time() - t_anchor
    los_pct     = 100 * los_arr[our_xi, our_yi, our_zi].sum() / n_local
    valid_n     = int(np.sum(~np.isnan(range_arr[our_xi, our_yi, our_zi])))
    print(f"  anchor_{anc_idx} ({label}) done  "
          f"LOS={los_pct:.1f}%  valid={valid_n}/{n_local}  "
          f"time={anchor_time:.0f}s", flush=True)

    if half_mode:
        # Write partial results to temp numpy file; parent merges after both halves.
        half_idx = 0 if pt_start == 0 else 1
        tmp_path = f"{output_path}.tmp_a{anc_idx}_h{half_idx}.npz"
        np.savez_compressed(tmp_path,
                            range_arr=range_arr,
                            los_arr=los_arr,
                            fp_power_arr=fp_power_arr,
                            tp_power_arr=tp_power_arr)
        print(f"  Temp results saved: {tmp_path}", flush=True)
    else:
        # Full-anchor mode: write anchor group directly to main HDF5.
        variance_arr = compute_variance(fp_power_arr)
        OPTS = {"compression": "gzip", "compression_opts": 4, "dtype": "float32"}
        with h5py.File(output_path, "a") as f_h5:
            pa_grp = f_h5.require_group("per_anchor")
            g      = pa_grp.create_group(f"anchor_{anc_idx}")
            g.create_dataset("range_m",             data=range_arr,    **OPTS)
            g.create_dataset("first_path_power_db", data=fp_power_arr, **OPTS)
            g.create_dataset("total_rx_power_db",   data=tp_power_arr, **OPTS)
            g.create_dataset("variance_m2",         data=variance_arr, **OPTS)
            g.create_dataset("los",                 data=los_arr)


# ── parent: merge two half-anchor temp files into the main HDF5 ───────────────

def _merge_halves(output_path: str, anc_idx: int):
    """
    Load the two .npz files written by the half-anchor children,
    combine them (each covers disjoint grid cells), compute variance
    on the merged array, and write the complete anchor group to HDF5.
    Deletes the temp files on success.
    """
    tmp0 = f"{output_path}.tmp_a{anc_idx}_h0.npz"
    tmp1 = f"{output_path}.tmp_a{anc_idx}_h1.npz"

    d0   = np.load(tmp0)
    d1   = np.load(tmp1)

    # Each half filled NaN/False into the other half's cells → combine with coalesce.
    range_arr    = np.where(~np.isnan(d0["range_arr"]),    d0["range_arr"],    d1["range_arr"])
    los_arr      = d0["los_arr"]    | d1["los_arr"]
    fp_power_arr = np.where(~np.isnan(d0["fp_power_arr"]), d0["fp_power_arr"], d1["fp_power_arr"])
    tp_power_arr = np.where(~np.isnan(d0["tp_power_arr"]), d0["tp_power_arr"], d1["tp_power_arr"])
    variance_arr = compute_variance(fp_power_arr)

    OPTS = {"compression": "gzip", "compression_opts": 4, "dtype": "float32"}
    with h5py.File(output_path, "a") as f_h5:
        pa_grp = f_h5.require_group("per_anchor")
        g      = pa_grp.create_group(f"anchor_{anc_idx}")
        g.create_dataset("range_m",             data=range_arr,    **OPTS)
        g.create_dataset("first_path_power_db", data=fp_power_arr, **OPTS)
        g.create_dataset("total_rx_power_db",   data=tp_power_arr, **OPTS)
        g.create_dataset("variance_m2",         data=variance_arr, **OPTS)
        g.create_dataset("los",                 data=los_arr)

    os.remove(tmp0)
    os.remove(tmp1)

    los_pct = 100 * los_arr.sum() / los_arr.size
    valid_n = int(np.sum(~np.isnan(range_arr)))
    print(f"  anchor_{anc_idx} merged  LOS={los_pct:.1f}%  "
          f"valid={valid_n}/{los_arr.size}", flush=True)


# ── parent orchestrator ───────────────────────────────────────────────────────

def parent_main(args, xs, ys, zs, anchors, output_path, half_mode: bool):
    NX, NY, NZ  = len(xs), len(ys), len(zs)
    total_pts   = NX * NY * NZ
    n_anchors   = len(anchors)
    n_spawns    = n_anchors * (2 if half_mode else 1)
    mode_label  = "half-anchor (10 spawns)" if half_mode else "full-anchor (5 spawns)"

    out_dir = os.path.dirname(output_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    # Write HDF5 skeleton once; children append anchor groups.
    with h5py.File(output_path, "w") as f_h5:
        f_h5.attrs["sionna_version"]       = "1.2.1"
        f_h5.attrs["frequency_hz"]         = FREQ_HZ
        f_h5.attrs["max_depth"]            = MAX_DEPTH
        f_h5.attrs["generation_timestamp"] = datetime.datetime.utcnow().isoformat()
        f_h5.attrs["grid_origin"]          = [float(xs[0]), float(ys[0]), float(zs[0])]
        f_h5.attrs["grid_resolution"]      = [xs[1]-xs[0] if NX > 1 else 0.05,
                                               ys[1]-ys[0] if NY > 1 else 0.05,
                                               zs[1]-zs[0] if NZ > 1 else 0.05]
        f_h5.attrs["grid_shape"]           = [NX, NY, NZ]
        f_h5.attrs["chunk_size_used"]      = args.chunk_size

        anc_pos = np.array([[a["x"], a["y"], a["z"]] for a in anchors], dtype=np.float32)
        anc_ids = np.array([a["id"] for a in anchors], dtype=np.uint8)
        f_h5.create_dataset("anchors",    data=anc_pos)
        f_h5.create_dataset("anchor_ids", data=anc_ids)
        f_h5.create_group("per_anchor")

    print(f"HDF5 skeleton written: {output_path}")
    print(f"Mode: {mode_label}  |  total subprocesses: {n_spawns}")

    script_path = os.path.abspath(__file__)
    common_args = ["--output-path", output_path, "--chunk-size", str(args.chunk_size)]
    if args.dry_run:
        common_args.append("--dry-run")

    half = total_pts // 2
    durations = []
    t_total   = time.time()

    for anc_idx in range(n_anchors):
        print(f"\n{'='*60}")
        t0 = time.time()

        if half_mode:
            # Two sequential children, each covering half the flat grid.
            for h_idx, (pt_s, pt_e) in enumerate([(0, half), (half, total_pts)]):
                label = f"anchor_{anc_idx} half-{h_idx} [{pt_s}:{pt_e}]"
                print(f"Spawning {label} ...", flush=True)
                subprocess.run(
                    [sys.executable, script_path,
                     "--single-anchor", str(anc_idx),
                     "--pt-start",      str(pt_s),
                     "--pt-end",        str(pt_e)]
                    + common_args,
                    check=True,
                )
            # Merge temp .npz files → anchor group in main HDF5
            print(f"Merging halves for anchor_{anc_idx} ...", flush=True)
            _merge_halves(output_path, anc_idx)

        else:
            # Single child handles the full anchor.
            print(f"Spawning subprocess for anchor_{anc_idx} ...", flush=True)
            subprocess.run(
                [sys.executable, script_path,
                 "--single-anchor", str(anc_idx)]
                + common_args,
                check=True,
            )

        dur = time.time() - t0
        durations.append(dur)
        print(f"anchor_{anc_idx} done: {dur:.0f}s ({dur/60:.1f} min)", flush=True)

    total_elapsed = time.time() - t_total
    fsize_mb      = os.path.getsize(output_path) / 1e6

    print(f"\n{'='*60}")
    print(f"ALL ANCHORS COMPLETE  [{mode_label}]")
    print(f"  Total wall time : {total_elapsed:.0f}s ({total_elapsed/60:.1f} min)")
    for i, d in enumerate(durations):
        print(f"  anchor_{i}       : {d:.0f}s ({d/60:.1f} min)")
    print(f"  Output          : {output_path}  ({fsize_mb:.1f} MB)")
    if not args.dry_run:
        print("\nNext step: python scripts/validate_table.py")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Precompute Sionna CIR table")
    parser.add_argument("--output-path",  default=None)
    parser.add_argument("--chunk-size",   type=int, default=3000,
                        help="Receivers per PathSolver call (default 3000)")
    parser.add_argument("--dry-run",      action="store_true",
                        help="5×5×5 sub-grid validation run")
    parser.add_argument("--half-anchors", action="store_true",
                        help="10 subprocesses (two per anchor) instead of 5")
    # ── hidden child-mode flags ──────────────────────────────────────────────
    parser.add_argument("--single-anchor", type=int, default=None,
                        help=argparse.SUPPRESS)
    parser.add_argument("--pt-start",      type=int, default=0,
                        help=argparse.SUPPRESS)
    parser.add_argument("--pt-end",        type=int, default=None,
                        help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.dry_run:
        xs, ys, zs = make_grid(DRY_X, DRY_Y, DRY_Z)
        suffix = "dry_run"
    else:
        xs, ys, zs = make_grid(GRID_X, GRID_Y, GRID_Z)
        suffix = "full"

    output_path = args.output_path or os.path.join(DATA_DIR, f"room_cir_table_{suffix}.h5")
    anchors     = load_anchors()

    if args.single_anchor is not None:
        # ── Child mode ───────────────────────────────────────────────────────
        run_child_anchor(args.single_anchor, output_path, xs, ys, zs,
                         args.chunk_size, anchors,
                         pt_start=args.pt_start, pt_end=args.pt_end)
    else:
        # ── Parent mode ──────────────────────────────────────────────────────
        total_pts = len(xs) * len(ys) * len(zs)
        if args.dry_run:
            print(f"DRY RUN — grid {len(xs)}×{len(ys)}×{len(zs)} = {total_pts} pts per anchor")
        else:
            print(f"FULL SWEEP — grid {len(xs)}×{len(ys)}×{len(zs)} = "
                  f"{total_pts} pts × {len(anchors)} anchors = "
                  f"{len(anchors)*total_pts} total queries")
        print(f"Anchors: {len(anchors)}  Output: {output_path}")
        parent_main(args, xs, ys, zs, anchors, output_path,
                    half_mode=args.half_anchors)


if __name__ == "__main__":
    InteractionType = None
    main()
