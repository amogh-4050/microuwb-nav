"""Run this once with: python3 make_notebooks.py
Creates 01_scene_sanity.ipynb and 02_single_point_test.ipynb.
"""
import nbformat as nbf
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def md(src):
    c = nbf.v4.new_markdown_cell(src)
    return c


def code(src):
    c = nbf.v4.new_code_cell(src.strip())
    return c


def save(nb, name):
    path = os.path.join(HERE, name)
    with open(path, "w") as f:
        nbf.write(nb, f)
    print(f"  wrote {path}")


# ───────────────────────────────────────────────────────────── notebook 01 ──

nb01 = nbf.v4.new_notebook()
nb01.cells = [

md("# 01 — Scene Sanity Check\n\nVerify the Mitsuba scene loads correctly:\n- bounding box matches the Gazebo world (0,0,0)→(5,4,3)\n- all 6 surfaces present\n- ITU concrete material registered\n- visual render looks like a rectangular room"),

code("""\
import os, sys
import numpy as np
import matplotlib.pyplot as plt
import sionna
import sionna.rt as rt
from sionna.rt import Camera

print(f"Sionna version: {sionna.__version__}")
assert sionna.__version__ == "1.2.1", f"Expected Sionna 1.2.1, got {sionna.__version__}"
print("Version OK")
"""),

md("## Load scene"),

code("""\
scene_path = os.path.abspath("../scene/room.xml")
print(f"Scene XML: {scene_path}")
scene = rt.load_scene(scene_path)
print(f"Objects     : {list(scene.objects.keys())}")
print(f"Materials   : {list(scene.radio_materials.keys())}")
"""),

md("## Bounding box assertion\n\nThe room interior spans exactly (0,0,0) → (5,4,3),\nmatching the Gazebo world coordinate frame."),

code("""\
bbox = scene.mi_scene.bbox()
b_min = np.array(bbox.min)
b_max = np.array(bbox.max)

print(f"BBox min : {b_min}")
print(f"BBox max : {b_max}")

TOL = 0.01   # 1 cm tolerance
expected_min = np.array([0.0, 0.0, 0.0])
expected_max = np.array([5.0, 4.0, 3.0])

assert np.allclose(b_min, expected_min, atol=TOL), (
    f"BBox min mismatch: got {b_min}, expected {expected_min}.\\n"
    "Check PLY vertex coordinates in scene/gen_meshes.py")
assert np.allclose(b_max, expected_max, atol=TOL), (
    f"BBox max mismatch: got {b_max}, expected {expected_max}.\\n"
    "Check PLY vertex coordinates in scene/gen_meshes.py")

print("\\nBOUNDING BOX ASSERTION PASSED")
"""),

md("## Visual renders\n\nTwo camera angles: top-down and perspective corner view.\nAll six surfaces (floor, ceiling, 4 walls) should be visible."),

code("""\
import os
RENDERS = os.path.abspath("../renders")
os.makedirs(RENDERS, exist_ok=True)

# Top-down view
cam_top = Camera(position=[2.5, 2.0, 12.0], look_at=[2.5, 2.0, 1.5])
bmp_top = scene.render(camera=cam_top, num_samples=128, resolution=(600, 480),
                       return_bitmap=True)
img_top = np.array(bmp_top)
fig, ax = plt.subplots(figsize=(7, 5))
ax.imshow(img_top); ax.set_title("Top-down view (looking -Z)"); ax.axis("off")
plt.tight_layout()
out = os.path.join(RENDERS, "01_top_down.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
plt.close()
print(f"Saved: {out}")
"""),

code("""\
# Perspective corner view
cam_persp = Camera(position=[-3.0, -2.0, 5.0], look_at=[2.5, 2.0, 1.5])
bmp_persp = scene.render(camera=cam_persp, num_samples=128, resolution=(600, 480),
                         return_bitmap=True)
img_persp = np.array(bmp_persp)
fig, ax = plt.subplots(figsize=(7, 5))
ax.imshow(img_persp); ax.set_title("Perspective corner view"); ax.axis("off")
plt.tight_layout()
out = os.path.join(RENDERS, "01_perspective.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
plt.close()
print(f"Saved: {out}")
"""),

md("All assertions passed. Scene is geometrically correct. Proceed to `02_single_point_test.ipynb`."),

]

save(nb01, "01_scene_sanity.ipynb")


# ───────────────────────────────────────────────────────────── notebook 02 ──

nb02 = nbf.v4.new_notebook()
nb02.cells = [

md("# 02 — Single-Point CIR Sanity Test\n\n"
   "Place anchor_0 as TX and room centre as RX.\n"
   "Run Sionna RT to compute multipath CIR.\n"
   "Assert: first-path range ≈ Euclidean distance to within 1 cm (confirms LOS and correct coordinate frame)."),

code("""\
import os
import numpy as np
import yaml
import matplotlib.pyplot as plt
import sionna
import sionna.rt as rt
from sionna.rt import (PathSolver, Transmitter, Receiver,
                       PlanarArray, Camera, InteractionType)

print(f"Sionna {sionna.__version__}")
"""),

md("## Load scene and set frequency"),

code("""\
scene_path = os.path.abspath("../scene/room.xml")
scene = rt.load_scene(scene_path)
scene.frequency = 6.5e9          # DW3000 channel 5 centre frequency
print(f"Scene loaded  — frequency = {scene.frequency.item() / 1e9:.2f} GHz")
print(f"Wavelength    = {scene.wavelength.item() * 100:.2f} cm")
"""),

md("## Load anchor positions from config YAML"),

code("""\
anchors_yaml = os.path.abspath(
    "../../src/microuwb_bringup/config/anchors.yaml")
print(f"Reading: {anchors_yaml}")
with open(anchors_yaml, "r") as f:
    cfg = yaml.safe_load(f)

anchors = cfg["anchors"]
print("\\nLoaded anchors:")
for a in anchors:
    print(f"  anchor_{a['id']}  ({a['x']}, {a['y']}, {a['z']})")
"""),

md("## Place TX (anchor_0) and RX (room centre)"),

code("""\
a0 = anchors[0]
TX_POS = [float(a0["x"]), float(a0["y"]), float(a0["z"])]
RX_POS = [2.5, 2.0, 1.0]    # room centre at 1 m height

euclidean_dist = float(np.linalg.norm(np.array(TX_POS) - np.array(RX_POS)))

print(f"TX anchor_0 position : {TX_POS}")
print(f"RX room-centre       : {RX_POS}")
print(f"Euclidean distance   : {euclidean_dist:.6f} m")
"""),

md("## Configure antennas and add devices"),

code("""\
# Isotropic single-element arrays (DW3000 approximation)
iso_array = PlanarArray(num_rows=1, num_cols=1,
                        vertical_spacing=0.5, horizontal_spacing=0.5,
                        pattern="iso", polarization="V")
scene.tx_array = iso_array
scene.rx_array = iso_array

scene.add(Transmitter(name="tx", position=TX_POS))
scene.add(Receiver(name="rx",   position=RX_POS))
print("TX and RX added")
"""),

md("## Compute paths\n\n"
   "`max_depth=4` allows up to 4 reflections.\n"
   "`refraction=False` keeps walls opaque (appropriate for concrete at 6.5 GHz).\n"
   "`diffraction=True` captures edge effects around doorframes/corners in future scenes."),

code("""\
solver = PathSolver()
paths = solver(
    scene,
    max_depth=4,
    los=True,
    specular_reflection=True,
    diffuse_reflection=False,
    refraction=False,
    diffraction=True,
    samples_per_src=int(1e6),
)
print(f"tau shape          : {paths.tau.shape}")
print(f"valid shape        : {paths.valid.shape}")
print(f"interactions shape : {paths.interactions.shape}")
"""),

md("## Extract first-path delay and compute range"),

code("""\
# Convert to numpy and collapse singleton TX/RX/antenna dimensions
# tau   shape: (num_rx, num_tx, num_paths)       with synthetic_array=True
# valid shape: (num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths) always
tau_all   = np.array(paths.tau).squeeze()     # → (num_paths,)
valid_all = np.array(paths.valid).squeeze()   # → (num_paths,)
valid_mask = valid_all.astype(bool)

tau_valid  = tau_all[valid_mask]
num_valid  = int(valid_mask.sum())

print(f"Total path slots : {len(tau_all)}")
print(f"Valid paths      : {num_valid}")

if num_valid == 0:
    raise RuntimeError(
        "No valid paths found.  Check that TX/RX are not inside walls "
        "and that scene normals point into the room.")

first_idx     = int(np.argmin(tau_valid))
first_delay_s = float(tau_valid[first_idx])

SPEED_OF_LIGHT = 3e8  # m/s
first_delay_ns  = first_delay_s * 1e9
first_range_m   = first_delay_s * SPEED_OF_LIGHT
range_error_cm  = abs(first_range_m - euclidean_dist) * 100

print()
print(f"First-path delay   : {first_delay_ns:.4f} ns")
print(f"First-path range   : {first_range_m:.4f} m")
print(f"Euclidean distance : {euclidean_dist:.4f} m")
print(f"Range error        : {range_error_cm:.3f} cm")
"""),

md("## LOS detection and sanity assertion"),

code("""\
# interactions shape: (max_depth, num_rx[_ant], num_tx[_ant], num_paths)
# Collapse everything except axis-0 and axis-(-1)
inter = np.array(paths.interactions)
md_  = inter.shape[0]
np_  = inter.shape[-1]
inter_2d = inter.reshape(md_, -1, np_)[:, 0, :]      # (max_depth, num_paths)
inter_valid = inter_2d[:, valid_mask]                  # (max_depth, num_valid_paths)
first_path_inter = inter_valid[:, first_idx]           # (max_depth,)

# LOS path has NONE (=0) at every depth
los_flag = bool(np.all(first_path_inter == InteractionType.NONE))

print(f"First-path interactions : {first_path_inter.tolist()}")
print(f"LOS detected            : {los_flag}")

TOLERANCE_M = 0.01   # 1 cm
err_m = abs(first_range_m - euclidean_dist)

print()
if los_flag and err_m < TOLERANCE_M:
    print("SANITY CHECK PASSED")
    print(f"  LOS range error {err_m * 100:.3f} cm is within {TOLERANCE_M * 100:.0f} cm tolerance")
else:
    msg_parts = []
    if not los_flag:
        msg_parts.append(
            "First path is NOT classified as LOS — possible wrong surface normals "
            "(check gen_meshes.py winding) or TX/RX inside a wall.")
    if err_m >= TOLERANCE_M:
        msg_parts.append(
            f"Range error {err_m:.4f} m exceeds {TOLERANCE_M} m tolerance — "
            "possible coordinate-frame mismatch between scene XML and anchors.yaml.")
    for msg in msg_parts:
        print(f"FAIL: {msg}")
    raise AssertionError("\\n".join(msg_parts))
"""),

md("## Path visualisation"),

code("""\
import os
RENDERS = os.path.abspath("../renders")
os.makedirs(RENDERS, exist_ok=True)

cam = Camera(position=[-3.0, -2.0, 5.0], look_at=[2.5, 2.0, 1.5])
bmp = scene.render(camera=cam, paths=paths, num_samples=128,
                   resolution=(720, 540), show_devices=True,
                   return_bitmap=True)
img = np.array(bmp)
fig, ax = plt.subplots(figsize=(9, 6))
ax.imshow(img); ax.set_title("Multipath from anchor_0 to room centre"); ax.axis("off")
plt.tight_layout()
out = os.path.join(RENDERS, "02_paths.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
plt.close()
print(f"Saved: {out}")
"""),

md("## Summary\n\n"
   "| Metric | Value |\n"
   "|---|---|\n"
   "| TX (anchor_0) | (0.1, 0.1, 2.5) |\n"
   "| RX (room centre) | (2.5, 2.0, 1.0) |\n"
   "| Frequency | 6.5 GHz |\n"
   "| First-path range | ≈ Euclidean distance |\n"
   "| LOS | True |\n\n"
   "Step 3a-ii will run this computation over a 2D grid and save to HDF5."),

]

save(nb02, "02_single_point_test.ipynb")

print("Done.")
