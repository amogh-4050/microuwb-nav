"""
Validate the CIR HDF5 lookup table produced by precompute_cir_table.py.

Usage:
    # Validate full table (default path):
    python scripts/validate_table.py

    # Validate dry-run table:
    python scripts/validate_table.py --input data/room_cir_table_dry_run.h5

Plots saved to: data/validation/
Exits with code 1 if any hard check fails.
"""
import argparse
import os
import sys

import h5py
import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PRECOMPUTE_DIR = os.path.dirname(SCRIPT_DIR)
DATA_DIR       = os.path.join(PRECOMPUTE_DIR, "data")
VAL_DIR        = os.path.join(DATA_DIR, "validation")

# Hard-fail thresholds
MAX_LOS_RANGE_ERROR_CM  = 10.0   # LOS cells: ray range must be ≈ Euclidean
MAX_NLOS_UNDER_EUCLID_CM = 0.0   # NLOS cells: range must be ≥ Euclidean (physics)
MAX_NLOS_PCT             = 50.0  # >50% NLOS across full grid → geometry problem


def fail(msg: str):
    print(f"\n[FAIL] {msg}")
    sys.exit(1)


def load_table(path: str):
    f = h5py.File(path, "r")
    return f


def euclid_grid(xs, ys, zs, anchor_pos):
    """(NX, NY, NZ) Euclidean distance array from anchor_pos."""
    ax, ay, az = anchor_pos
    XX, YY, ZZ = np.meshgrid(xs, ys, zs, indexing="ij")
    return np.sqrt((XX - ax)**2 + (YY - ay)**2 + (ZZ - az)**2).astype(np.float32)


def restore_grid(f, attr_name):
    origin     = np.array(f.attrs["grid_origin"])
    resolution = np.array(f.attrs["grid_resolution"])
    shape      = np.array(f.attrs["grid_shape"], dtype=int)
    xs = origin[0] + np.arange(shape[0]) * resolution[0]
    ys = origin[1] + np.arange(shape[1]) * resolution[1]
    zs = origin[2] + np.arange(shape[2]) * resolution[2]
    return xs, ys, zs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default=None,
                        help="HDF5 file to validate (default: data/room_cir_table_full.h5)")
    args = parser.parse_args()

    path = args.input or os.path.join(DATA_DIR, "room_cir_table_full.h5")
    if not os.path.exists(path):
        # Fallback to dry-run table for convenience
        dry = os.path.join(DATA_DIR, "room_cir_table_dry_run.h5")
        if os.path.exists(dry):
            path = dry
            print(f"Full table not found, falling back to: {path}")
        else:
            print(f"File not found: {path}")
            sys.exit(1)

    os.makedirs(VAL_DIR, exist_ok=True)
    fsize_mb = os.path.getsize(path) / 1e6
    print(f"Validating: {path}  ({fsize_mb:.1f} MB)")

    f = load_table(path)

    # ── Print metadata ────────────────────────────────────────────────────────
    print(f"\n── Metadata ──")
    for k, v in f.attrs.items():
        print(f"  {k}: {v}")

    xs, ys, zs = restore_grid(f, "grid_origin")
    grid_shape = tuple(f.attrs["grid_shape"].astype(int))
    n_anchors  = len(f["anchors"])
    anc_pos    = np.array(f["anchors"])
    total_pts  = int(np.prod(grid_shape))

    print(f"\n── Grid ──")
    print(f"  shape:      {grid_shape}  = {total_pts} points")
    print(f"  X: {xs[0]:.2f} → {xs[-1]:.2f}  ({len(xs)} pts)")
    print(f"  Y: {ys[0]:.2f} → {ys[-1]:.2f}  ({len(ys)} pts)")
    print(f"  Z: {zs[0]:.2f} → {zs[-1]:.2f}  ({len(zs)} pts)")
    print(f"  anchors:    {n_anchors}")

    hard_fails = []

    for ai in range(n_anchors):
        grp   = f[f"per_anchor/anchor_{ai}"]
        r_arr = np.array(grp["range_m"])
        los_a = np.array(grp["los"]).astype(bool)
        fp_db = np.array(grp["first_path_power_db"])
        tp_db = np.array(grp["total_rx_power_db"])
        var_a = np.array(grp["variance_m2"])
        euclid = euclid_grid(xs, ys, zs, anc_pos[ai])

        valid     = ~np.isnan(r_arr)
        los_mask  = valid & los_a
        nlos_mask = valid & ~los_a

        los_pct  = 100 * los_mask.sum() / total_pts
        nlos_pct = 100 * nlos_mask.sum() / total_pts
        valid_pct = 100 * valid.sum() / total_pts

        print(f"\n── anchor_{ai} @ {anc_pos[ai].tolist()} ──")
        print(f"  valid cells : {valid.sum()}/{total_pts}  ({valid_pct:.1f}%)")
        print(f"  LOS         : {los_mask.sum()}  ({los_pct:.1f}%)")
        print(f"  NLOS        : {nlos_mask.sum()}  ({nlos_pct:.1f}%)")

        # ── Check 1: no more than 50% NLOS across full grid ──────────────────
        if los_pct < (100 - MAX_NLOS_PCT) and valid.sum() > 100:
            hard_fails.append(
                f"anchor_{ai}: only {los_pct:.1f}% LOS — suggests geometry error")

        # ── Check 2: LOS cells — range error vs Euclidean < 10 cm ───────────
        if los_mask.sum() > 0:
            err_los_cm = np.abs(r_arr[los_mask] - euclid[los_mask]) * 100
            median_err = float(np.nanmedian(err_los_cm))
            p95_err    = float(np.nanpercentile(err_los_cm, 95))
            print(f"  LOS range error: median={median_err:.3f} cm  p95={p95_err:.3f} cm")
            if p95_err > MAX_LOS_RANGE_ERROR_CM:
                hard_fails.append(
                    f"anchor_{ai}: LOS range error p95={p95_err:.1f} cm "
                    f"exceeds {MAX_LOS_RANGE_ERROR_CM} cm")

        # ── Check 3: NLOS cells — range must be ≥ Euclidean ─────────────────
        if nlos_mask.sum() > 0:
            range_diff = r_arr[nlos_mask] - euclid[nlos_mask]
            n_under = int((range_diff < -0.01).sum())
            print(f"  NLOS range excess: median={float(np.nanmedian(range_diff))*100:.1f} cm")
            if n_under > 0:
                pct_under = 100 * n_under / nlos_mask.sum()
                msg = (f"anchor_{ai}: {n_under} NLOS cells ({pct_under:.1f}%) "
                       f"have range < Euclidean — first-path extraction bug?")
                if pct_under > 1.0:
                    hard_fails.append(msg)
                else:
                    print(f"  [WARN] {msg}")

        # ── Plots ─────────────────────────────────────────────────────────────
        _plot_anchor(ai, anc_pos[ai], r_arr, los_a, fp_db, tp_db,
                     euclid, xs, ys, zs, grid_shape, VAL_DIR)

    f.close()

    # ── Final summary ─────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    if hard_fails:
        print("  VALIDATION FAILED")
        for m in hard_fails:
            print(f"  [FAIL] {m}")
        sys.exit(1)
    else:
        print("  VALIDATION PASSED — all checks OK")
        print(f"  Plots saved to: {VAL_DIR}")
    print("=" * 60)


def _plot_anchor(ai, anc_pos, r_arr, los_a, fp_db, tp_db,
                 euclid, xs, ys, zs, grid_shape, out_dir):
    NX, NY, NZ = grid_shape
    valid = ~np.isnan(r_arr)
    los_m = valid & los_a.astype(bool)
    nlos_m = valid & ~los_a.astype(bool)

    # ── LOS coverage map at z = 1.5 m (nearest slice) ───────────────────────
    z_target = 1.5
    zi = int(np.argmin(np.abs(zs - z_target)))

    los_slice = los_a[:, :, zi].astype(float)
    valid_slice = valid[:, :, zi]
    los_slice[~valid_slice] = np.nan

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    im = axes[0].imshow(
        los_slice.T, origin="lower", aspect="auto",
        extent=[xs[0], xs[-1], ys[0], ys[-1]],
        cmap="RdYlGn", vmin=0, vmax=1)
    axes[0].scatter(anc_pos[0], anc_pos[1], c="blue", s=60, marker="^",
                    zorder=5, label=f"anchor_{ai}")
    axes[0].set_xlabel("X (m)"); axes[0].set_ylabel("Y (m)")
    axes[0].set_title(f"LOS coverage — anchor_{ai} at z≈{zs[zi]:.2f}m slice")
    axes[0].legend(fontsize=8)
    plt.colorbar(im, ax=axes[0], label="LOS (1) / NLOS (0)")
    los_pct = 100 * np.nansum(los_slice) / np.sum(valid_slice)
    axes[0].set_xlabel(f"X (m)   LOS={los_pct:.0f}%")

    # ── Range error histogram (LOS) and excess histogram (NLOS) ─────────────
    ax = axes[1]
    if los_m.sum() > 0:
        err = (r_arr[los_m] - euclid[los_m]) * 100
        ax.hist(np.clip(err, -5, 5), bins=50, alpha=0.7,
                label=f"LOS (n={los_m.sum()})", color="steelblue")
    if nlos_m.sum() > 0:
        excess = np.clip((r_arr[nlos_m] - euclid[nlos_m]) * 100, 0, 100)
        ax.hist(excess, bins=50, alpha=0.7,
                label=f"NLOS excess (n={nlos_m.sum()})", color="darkorange")
    ax.set_xlabel("Range error / excess (cm)")
    ax.set_ylabel("Count")
    ax.set_title(f"anchor_{ai}: range errors")
    ax.legend(fontsize=8)

    plt.tight_layout()
    out = os.path.join(out_dir, f"anchor_{ai}_los_and_errors.png")
    plt.savefig(out, dpi=120, bbox_inches="tight")
    plt.close()
    print(f"  Saved: {out}")

    # ── First-path power heatmap at z ≈ 1.5m ─────────────────────────────────
    fp_slice = fp_db[:, :, zi].copy()
    fp_slice[~valid_slice] = np.nan

    fig, ax = plt.subplots(figsize=(7, 5))
    vmin = np.nanpercentile(fp_slice[valid_slice], 5)
    vmax = np.nanpercentile(fp_slice[valid_slice], 95)
    im = ax.imshow(fp_slice.T, origin="lower", aspect="auto",
                   extent=[xs[0], xs[-1], ys[0], ys[-1]],
                   cmap="plasma", vmin=vmin, vmax=vmax)
    ax.scatter(anc_pos[0], anc_pos[1], c="cyan", s=60, marker="^", zorder=5)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"anchor_{ai}: first-path power (dB) at z≈{zs[zi]:.2f}m")
    plt.colorbar(im, ax=ax, label="Power (dB)")
    plt.tight_layout()
    out = os.path.join(out_dir, f"anchor_{ai}_fp_power_heatmap.png")
    plt.savefig(out, dpi=120, bbox_inches="tight")
    plt.close()
    print(f"  Saved: {out}")


if __name__ == "__main__":
    main()
