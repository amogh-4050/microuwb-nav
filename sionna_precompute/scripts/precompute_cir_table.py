"""
Sionna RT grid sweep — generates CIR lookup table for all 5 anchors.

Usage:
    # Validate first (5x5x5 sub-grid, ~1 min):
    python scripts/precompute_cir_table.py --dry-run

    # Full sweep (~5-30 min on RTX 4060):
    python scripts/precompute_cir_table.py

    # Tune chunk size if OOM (default 10000):
    python scripts/precompute_cir_table.py --chunk-size 5000

Multi-RX batching strategy: PathSolver supports multiple receivers in one call.
We add chunk_size Receiver objects, run the solver once, then remove them.
This is faster than per-point calls because ray launches scale with TX count,
not RX count — more RXs just adds hit-checking overhead per ray bounce.
"""
import argparse
import datetime
import os
import sys
import time

import h5py
import numpy as np
import yaml

# ── locate repo root regardless of cwd ───────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PRECOMPUTE_DIR = os.path.dirname(SCRIPT_DIR)
REPO_ROOT = os.path.dirname(PRECOMPUTE_DIR)

SCENE_XML    = os.path.join(PRECOMPUTE_DIR, "scene", "room.xml")
ANCHORS_YAML = os.path.join(REPO_ROOT, "src", "microuwb_bringup", "config", "anchors.yaml")
DATA_DIR     = os.path.join(PRECOMPUTE_DIR, "data")

# ── constants ─────────────────────────────────────────────────────────────────
C        = 3e8          # speed of light m/s
FREQ_HZ  = 6.5e9       # DW3000 channel 5
MAX_DEPTH = 4
SAMPLES  = int(1e6)

# Grid bounds and resolution (full sweep)
GRID_X = (0.20, 4.80, 0.05)   # start, stop (inclusive), step
GRID_Y = (0.20, 3.80, 0.05)
GRID_Z = (0.10, 2.90, 0.05)

# Dry-run sub-grid
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


def try_remove(scene, name: str):
    try:
        scene.remove(name)
    except Exception:
        pass


def extract_metrics(paths, N_rx: int):
    """
    Extract per-receiver first-path metrics from a Sionna Paths object.

    With synthetic_array=True (Sionna 1.2.1 default):
      paths.tau          : (N_rx, 1, n_paths)
      paths.valid        : (N_rx, 1, n_paths)
      paths.interactions : (max_depth, N_rx, 1, n_paths)
      paths.a            : complex, (N_rx, 1, n_paths) — try/except guarded

    Returns four arrays of length N_rx:
      range_m, los_flag, first_path_power_db, total_rx_power_db
    """
    NONE = InteractionType.NONE

    tau_np   = np.array(paths.tau)      # (N_rx, 1, n_paths)
    valid_np = np.array(paths.valid).astype(bool)   # same shape

    # Collapse n_tx=1 dim → (N_rx, n_paths)
    if tau_np.ndim == 3:
        tau_np   = tau_np[:, 0, :]
        valid_np = valid_np[:, 0, :]
    elif tau_np.ndim == 2:
        pass  # already (N_rx, n_paths)

    inter_np = np.array(paths.interactions)  # (max_depth, N_rx, 1, n_paths)
    if inter_np.ndim == 4:
        inter_np = inter_np[:, :, 0, :]      # → (max_depth, N_rx, n_paths)

    # Try to get complex amplitudes for power
    try:
        a_raw = np.array(paths.a)
        power_np = np.abs(a_raw) ** 2
        # Collapse to (N_rx, n_paths): squeeze all 1-dims except first+last
        while power_np.ndim > 2 and power_np.shape[1] == 1:
            power_np = power_np[:, 0, ...]
        if power_np.ndim == 3 and power_np.shape[-1] == 1:
            power_np = power_np[..., 0]
        has_power = power_np.ndim == 2 and power_np.shape[0] == N_rx
    except Exception:
        has_power = False

    range_m        = np.full(N_rx, np.nan, dtype=np.float32)
    los_flag       = np.zeros(N_rx, dtype=bool)
    first_power_db = np.full(N_rx, np.nan, dtype=np.float32)
    total_power_db = np.full(N_rx, np.nan, dtype=np.float32)

    EPS = 1e-30

    for i in range(N_rx):
        valid_i = valid_np[i]
        if not valid_i.any():
            continue

        tau_i = tau_np[i]
        tau_valid = tau_i[valid_i]
        local_first = int(np.argmin(tau_valid))
        global_first = int(np.where(valid_i)[0][local_first])

        range_m[i] = float(tau_valid[local_first]) * C

        # LOS: all interaction types NONE at every depth
        inter_i = inter_np[:, i, global_first]
        los_flag[i] = bool(np.all(inter_i == NONE))

        if has_power:
            p_i = power_np[i]
            p_valid = p_i[valid_i]
            fp = float(p_valid[local_first])
            tp = float(p_valid.sum())
            first_power_db[i] = float(10 * np.log10(max(fp, EPS)))
            total_power_db[i] = float(10 * np.log10(max(tp, EPS)))

    return range_m, los_flag, first_power_db, total_power_db


def compute_variance(first_power_db: np.ndarray) -> np.ndarray:
    """
    Variance heuristic: inversely proportional to first-path power.

    Maps highest-power cells → 0.001 m² (≈1 cm std, good LOS).
    Maps lowest-power cells  → 0.050 m² (≈22 cm std, poor NLOS).
    No-path cells (NaN)      → 0.250 m² (maximum uncertainty).

    Scale is relative (per-anchor normalised) so the Kalman filter in step 5
    must apply an absolute calibration factor before use.
    """
    valid = ~np.isnan(first_power_db)
    fp_linear = np.where(valid, 10.0 ** (first_power_db / 10.0), 0.0)
    p_max = fp_linear[valid].max() if valid.any() else 1.0
    power_norm = np.clip(fp_linear / max(p_max, 1e-30), 0.0, 1.0)
    variance = np.where(valid, 0.001 + 0.049 * (1.0 - power_norm), 0.25)
    return variance.astype(np.float32)


def run_sweep(output_path: str, xs, ys, zs, chunk_size: int, anchors: list,
              scene, solver):
    from sionna.rt import Transmitter, Receiver

    NX, NY, NZ = len(xs), len(ys), len(zs)
    grid_shape = (NX, NY, NZ)
    total_pts  = NX * NY * NZ

    # Pre-build flat grid
    xi_idx, yi_idx, zi_idx = np.meshgrid(np.arange(NX), np.arange(NY),
                                          np.arange(NZ), indexing="ij")
    xi_flat = xi_idx.ravel()
    yi_flat = yi_idx.ravel()
    zi_flat = zi_idx.ravel()
    pts_flat = np.stack([xs[xi_flat], ys[yi_flat], zs[zi_flat]], axis=1)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with h5py.File(output_path, "w") as f_h5:
        # File-level attributes
        f_h5.attrs["sionna_version"]       = "1.2.1"
        f_h5.attrs["frequency_hz"]         = FREQ_HZ
        f_h5.attrs["max_depth"]            = MAX_DEPTH
        f_h5.attrs["generation_timestamp"] = datetime.datetime.utcnow().isoformat()
        f_h5.attrs["grid_origin"]          = [float(xs[0]), float(ys[0]), float(zs[0])]
        f_h5.attrs["grid_resolution"]      = [xs[1]-xs[0] if len(xs)>1 else 0.05,
                                               ys[1]-ys[0] if len(ys)>1 else 0.05,
                                               zs[1]-zs[0] if len(zs)>1 else 0.05]
        f_h5.attrs["grid_shape"]           = list(grid_shape)
        f_h5.attrs["chunk_size_used"]      = chunk_size

        # Anchor metadata
        anc_pos = np.array([[a["x"], a["y"], a["z"]] for a in anchors], dtype=np.float32)
        anc_ids = np.array([a["id"] for a in anchors], dtype=np.uint8)
        f_h5.create_dataset("anchors",    data=anc_pos)
        f_h5.create_dataset("anchor_ids", data=anc_ids)

        pa_grp = f_h5.create_group("per_anchor")

        for anc_idx, anchor in enumerate(anchors):
            anc_pos_i = [float(anchor["x"]), float(anchor["y"]), float(anchor["z"])]
            print(f"\n[anchor_{anc_idx}] TX at {anc_pos_i} — {total_pts} points "
                  f"in {(total_pts + chunk_size - 1) // chunk_size} chunks")

            # Pre-allocate result arrays
            range_arr    = np.full(grid_shape, np.nan, dtype=np.float32)
            los_arr      = np.zeros(grid_shape, dtype=bool)
            fp_power_arr = np.full(grid_shape, np.nan, dtype=np.float32)
            tp_power_arr = np.full(grid_shape, np.nan, dtype=np.float32)

            try_remove(scene, "tx")
            scene.add(Transmitter(name="tx", position=anc_pos_i))

            n_chunks  = (total_pts + chunk_size - 1) // chunk_size
            t_anchor  = time.time()

            for ci in range(n_chunks):
                start = ci * chunk_size
                end   = min(start + chunk_size, total_pts)
                chunk = pts_flat[start:end]
                N     = len(chunk)

                rx_names = [f"rx_{k}" for k in range(N)]
                for k, pt in enumerate(chunk):
                    scene.add(Receiver(name=rx_names[k], position=pt.tolist()))

                p = solver(scene, max_depth=MAX_DEPTH, los=True,
                           specular_reflection=True, diffuse_reflection=False,
                           refraction=False, diffraction=True,
                           samples_per_src=SAMPLES)

                r_chunk, l_chunk, fp_chunk, tp_chunk = extract_metrics(p, N)

                for local_i in range(N):
                    gi   = start + local_i
                    xi_  = int(xi_flat[gi])
                    yi_  = int(yi_flat[gi])
                    zi_  = int(zi_flat[gi])
                    range_arr[xi_, yi_, zi_]    = r_chunk[local_i]
                    los_arr[xi_, yi_, zi_]      = l_chunk[local_i]
                    fp_power_arr[xi_, yi_, zi_] = fp_chunk[local_i]
                    tp_power_arr[xi_, yi_, zi_] = tp_chunk[local_i]

                for name in rx_names:
                    try_remove(scene, name)

                if ci % 10 == 0 or ci == n_chunks - 1:
                    elapsed = time.time() - t_anchor
                    done = end / total_pts
                    eta  = elapsed / max(done, 1e-9) * (1 - done)
                    print(f"  chunk {ci+1:4d}/{n_chunks}  "
                          f"{100*done:5.1f}%  "
                          f"elapsed {elapsed:.0f}s  ETA {eta:.0f}s")

            variance_arr = compute_variance(fp_power_arr)

            g = pa_grp.create_group(f"anchor_{anc_idx}")
            OPTS = {"compression": "gzip", "compression_opts": 4, "dtype": "float32"}
            g.create_dataset("range_m",               data=range_arr,    **OPTS)
            g.create_dataset("first_path_power_db",   data=fp_power_arr, **OPTS)
            g.create_dataset("total_rx_power_db",     data=tp_power_arr, **OPTS)
            g.create_dataset("variance_m2",           data=variance_arr, **OPTS)
            g.create_dataset("los",                   data=los_arr)
            f_h5.flush()

            los_pct = 100 * los_arr.sum() / total_pts
            valid_n = np.sum(~np.isnan(range_arr))
            print(f"  anchor_{anc_idx} done  LOS={los_pct:.1f}%  "
                  f"valid={valid_n}/{total_pts}  "
                  f"time={time.time()-t_anchor:.0f}s")

    fsize_mb = os.path.getsize(output_path) / 1e6
    print(f"\nWrote: {output_path}  ({fsize_mb:.1f} MB)")
    return output_path


def main():
    parser = argparse.ArgumentParser(description="Precompute Sionna CIR table")
    parser.add_argument("--output-path", default=None,
                        help="HDF5 output path (default: data/room_cir_table.h5)")
    parser.add_argument("--chunk-size", type=int, default=10000,
                        help="Receivers per PathSolver call (tune for VRAM)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run on 5x5x5 sub-grid to validate script (~1 min)")
    args = parser.parse_args()

    if args.chunk_size < 1000:
        print(f"WARNING: chunk_size={args.chunk_size} is very low. "
              "If you're hitting OOM, investigate root cause before reducing further.")

    if args.dry_run:
        xs, ys, zs = make_grid(DRY_X, DRY_Y, DRY_Z)
        suffix = "dry_run"
        print(f"DRY RUN — grid {len(xs)}×{len(ys)}×{len(zs)} = "
              f"{len(xs)*len(ys)*len(zs)} points")
    else:
        xs, ys, zs = make_grid(GRID_X, GRID_Y, GRID_Z)
        suffix = "full"
        print(f"FULL SWEEP — grid {len(xs)}×{len(ys)}×{len(zs)} = "
              f"{len(xs)*len(ys)*len(zs)} points × 5 anchors = "
              f"{5*len(xs)*len(ys)*len(zs)} total queries")

    output_path = args.output_path or os.path.join(
        DATA_DIR, f"room_cir_table_{suffix}.h5")

    anchors = load_anchors()
    print(f"Anchors loaded: {len(anchors)}")
    print(f"Output: {output_path}")

    # Import Sionna only after args are parsed (avoids slow TF import on --help)
    import sionna.rt as rt
    from sionna.rt import PathSolver, InteractionType as _IT

    # Patch InteractionType into module scope for extract_metrics
    global InteractionType
    InteractionType = _IT

    print("Loading scene …")
    scene = rt.load_scene(SCENE_XML)
    scene.frequency = FREQ_HZ

    iso_array = rt.PlanarArray(num_rows=1, num_cols=1,
                               vertical_spacing=0.5, horizontal_spacing=0.5,
                               pattern="iso", polarization="V")
    scene.tx_array = iso_array
    scene.rx_array = iso_array

    solver = PathSolver()

    t_start = time.time()
    run_sweep(output_path, xs, ys, zs, args.chunk_size, anchors, scene, solver)
    total_time = time.time() - t_start

    print(f"\nTotal wall time: {total_time:.0f}s ({total_time/60:.1f} min)")

    if not args.dry_run:
        print("\nNext step: python scripts/validate_table.py")


if __name__ == "__main__":
    # Declare InteractionType at module scope so extract_metrics can use it
    InteractionType = None
    main()
