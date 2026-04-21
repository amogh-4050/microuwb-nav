#!/usr/bin/env python3
"""One-shot fix: replace variance_m2 in room_cir_table_full.h5 with LOS-flag heuristic.

LOS  → 0.005 m²  (~7cm std dev, DW3000-realistic)
NLOS → 0.050 m²  (~22cm std dev, DW3000-realistic)

Safe to re-run; the operation is idempotent.
"""

import sys
from pathlib import Path

import h5py
import numpy as np

HDF5_PATH = (
    Path(__file__).resolve().parents[2]
    / "sionna_precompute" / "data" / "room_cir_table_full.h5"
)

VAR_LOS  = 0.005   # m²
VAR_NLOS = 0.050   # m²


def main() -> None:
    if not HDF5_PATH.exists():
        print(f"ERROR: HDF5 not found at {HDF5_PATH}", file=sys.stderr)
        sys.exit(1)

    print(f"Opening {HDF5_PATH} in r+ mode …")
    with h5py.File(HDF5_PATH, "r+") as f:
        anchor_ids = f["anchor_ids"][...]
        for aid in anchor_ids:
            key = f"anchor_{aid}"
            grp = f["per_anchor"][key]

            los      = grp["los"][...]           # bool array (nx, ny, nz)
            variance = grp["variance_m2"]        # h5py Dataset — write back in place

            new_var = np.where(los, VAR_LOS, VAR_NLOS).astype(np.float32)
            variance[...] = new_var

            n_los  = int(los.sum())
            n_nlos = int((~los).sum())
            print(
                f"  anchor_{aid}: {n_los:>7} LOS cells → {VAR_LOS}  |  "
                f"{n_nlos:>6} NLOS cells → {VAR_NLOS}"
            )

    print("Done — variance_m2 updated for all anchors.")


if __name__ == "__main__":
    main()
