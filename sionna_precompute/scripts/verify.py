import h5py
import numpy as np

from pathlib import Path
_here = Path(__file__).parent
f = h5py.File(_here.parent / 'data' / 'room_cir_table_full.h5', 'r')

# Pick a flight-altitude point roughly in the center
origin = f.attrs['grid_origin']
res = f.attrs['grid_resolution']
test_pt = np.array([2.5, 2.0, 1.5])
idx = tuple(((test_pt - origin) / res).astype(int))
print(f"Testing grid point {idx} → world {test_pt}")

for a in range(5):
    anc_pos = f['anchors'][a]
    euclid = np.linalg.norm(anc_pos - test_pt)
    r = f[f'per_anchor/anchor_{a}/range_m'][idx]
    los = f[f'per_anchor/anchor_{a}/los'][idx]
    fp = f[f'per_anchor/anchor_{a}/first_path_power_db'][idx]
    var = f[f'per_anchor/anchor_{a}/variance_m2'][idx]
    print(f"  anc{a}: euclid={euclid:.3f}m  sim={r:.3f}m  "
          f"los={los}  fp={fp:.1f}dB  var={var:.4f}")
f.close()