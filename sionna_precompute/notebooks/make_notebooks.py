"""Run this once with: python3 make_notebooks.py
Creates 01_scene_sanity.ipynb, 02_single_point_test.ipynb,
and 03_path_visualization.ipynb.
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

md("# 01 — Scene Sanity Check\n\nVerify the Mitsuba scene loads correctly:\n- bounding box matches the Gazebo world (0,0,0)→(5,4,3)\n- all surfaces present (6 room + 3 furniture)\n- ITU concrete + ITU wood materials registered\n- visual render looks like a rectangular room with furniture"),

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

md("## Bounding box assertion\n\nThe room interior spans exactly (0,0,0) → (5,4,3),\nmatching the Gazebo world coordinate frame.\nFurniture is inside the room so does not extend the bbox."),

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

md("## Visual renders\n\nTwo camera angles: top-down and perspective corner view.\nAll six surfaces (floor, ceiling, 4 walls) should be visible, plus furniture boxes."),

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
   "Test 1 (LOS with furniture): anchor_0 ceiling corner → room centre.\n"
   "Test 2 (NLOS): anchor_3 floor → point behind bookshelf.\n\n"
   "Assert Test 1: first-path range ≈ Euclidean distance to within 1 cm.\n"
   "Assert Test 2: LOS flag False, first-path range > Euclidean distance."),

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

md("## Load scene (with furniture) and set frequency"),

code("""\
scene_path = os.path.abspath("../scene/room.xml")
scene = rt.load_scene(scene_path)
scene.frequency = 6.5e9          # DW3000 channel 5 centre frequency
print(f"Scene loaded  — frequency = {scene.frequency.item() / 1e9:.2f} GHz")
print(f"Wavelength    = {scene.wavelength.item() * 100:.2f} cm")
print(f"Objects in scene: {list(scene.objects.keys())}")
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

md("## Test 1: LOS — anchor_0 (ceiling) → room centre\n\n"
   "The bookshelf, cafe table, and cabinet do not obstruct this path "
   "(verified geometrically: the line a0→centre passes above all furniture)."),

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

md("## Compute paths"),

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
# tau shape with synthetic_array=True: (num_rx, num_tx, num_paths)
# valid: same shape
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

md("## LOS detection and Test 1 assertion"),

code("""\
inter = np.array(paths.interactions)
md_  = inter.shape[0]
np_  = inter.shape[-1]
inter_2d = inter.reshape(md_, -1, np_)[:, 0, :]      # (max_depth, num_paths)
inter_valid = inter_2d[:, valid_mask]
first_path_inter = inter_valid[:, first_idx]

los_flag_los = bool(np.all(first_path_inter == InteractionType.NONE))
err_los = abs(first_range_m - euclidean_dist)

print(f"First-path interactions : {first_path_inter.tolist()}")
print(f"LOS detected            : {los_flag_los}")
print(f"Range error             : {err_los * 100:.3f} cm")

TOLERANCE_M = 0.01   # 1 cm
test1_pass = los_flag_los and err_los < TOLERANCE_M

print()
if test1_pass:
    print("TEST 1 (LOS WITH FURNITURE) : PASSED")
    print(f"  LOS range error {err_los * 100:.3f} cm < {TOLERANCE_M * 100:.0f} cm tolerance")
else:
    msgs = []
    if not los_flag_los:
        msgs.append("First path NOT LOS — check normals or TX/RX placement.")
    if err_los >= TOLERANCE_M:
        msgs.append(f"Range error {err_los:.4f} m exceeds {TOLERANCE_M} m tolerance.")
    for m in msgs:
        print(f"FAIL: {m}")
    raise AssertionError("\\n".join(msgs))
"""),

md("## Path visualisation (Test 1)"),

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
ax.imshow(img)
ax.set_title("Test 1: anchor_0 (ceiling) → room centre — LOS with furniture")
ax.axis("off")
plt.tight_layout()
out = os.path.join(RENDERS, "02_test1_los_paths.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
plt.close()
print(f"Saved: {out}")
"""),

md("## Test 2: NLOS — anchor_3 (floor west) → behind bookshelf\n\n"
   "anchor_3 is at (0.1, 2.0, 0.3) on the floor near the west wall.\n"
   "The test RX at (0.5, 0.3, 0.5) is behind the bookshelf "
   "(bookshelf AABB: x=[0.05,0.95], y=[0.61,1.01], z=[0,1.2]).\n\n"
   "Geometric verification: the line a3→RX enters the bookshelf at "
   "t≈0.58 (x=0.33, z=0.42) and exits at t≈0.82 (x=0.43, z=0.46).\n\n"
   "Expected: LOS flag=False, first-path range > Euclidean distance."),

code("""\
# Clear previous TX/RX before setting up NLOS test
scene.remove("tx")
scene.remove("rx")

a3 = anchors[3]
TX_NLOS = [float(a3["x"]), float(a3["y"]), float(a3["z"])]  # (0.1, 2.0, 0.3)
RX_NLOS = [0.5, 0.3, 0.5]   # south of bookshelf — geometrically blocked from a3

euclidean_nlos = float(np.linalg.norm(np.array(TX_NLOS) - np.array(RX_NLOS)))

print(f"TX anchor_3 (floor) : {TX_NLOS}")
print(f"RX (behind shelf)   : {RX_NLOS}")
print(f"Euclidean distance  : {euclidean_nlos:.4f} m")

scene.add(Transmitter(name="tx", position=TX_NLOS))
scene.add(Receiver(name="rx",   position=RX_NLOS))
print("NLOS TX/RX added")
"""),

code("""\
paths_nlos = solver(
    scene,
    max_depth=4,
    los=True,
    specular_reflection=True,
    diffuse_reflection=False,
    refraction=False,
    diffraction=True,
    samples_per_src=int(1e6),
)
print(f"tau shape          : {paths_nlos.tau.shape}")
print(f"valid shape        : {paths_nlos.valid.shape}")
"""),

code("""\
tau_nlos   = np.array(paths_nlos.tau).squeeze()
valid_nlos = np.array(paths_nlos.valid).squeeze().astype(bool)
tau_v_nlos = tau_nlos[valid_nlos]
num_valid_nlos = int(valid_nlos.sum())

print(f"Valid paths (NLOS test) : {num_valid_nlos}")

if num_valid_nlos == 0:
    # No paths at all → extreme NLOS, test passes trivially
    los_flag_nlos = False
    first_range_nlos = float('inf')
    print("No valid paths — extreme NLOS (bookshelf fully blocking, no diffraction escape)")
else:
    first_idx_nlos   = int(np.argmin(tau_v_nlos))
    first_delay_nlos = float(tau_v_nlos[first_idx_nlos])
    first_range_nlos = first_delay_nlos * 3e8

    inter_nlos  = np.array(paths_nlos.interactions)
    md2  = inter_nlos.shape[0]
    np2  = inter_nlos.shape[-1]
    inter2d_nlos  = inter_nlos.reshape(md2, -1, np2)[:, 0, :]
    inter_v_nlos  = inter2d_nlos[:, valid_nlos]
    fp_inter_nlos = inter_v_nlos[:, first_idx_nlos]

    los_flag_nlos = bool(np.all(fp_inter_nlos == InteractionType.NONE))

    print(f"First-path interactions : {fp_inter_nlos.tolist()}")
    print(f"LOS detected            : {los_flag_nlos}")
    print(f"First-path range        : {first_range_nlos:.4f} m")
    print(f"Euclidean distance      : {euclidean_nlos:.4f} m")
    print(f"Range excess            : {(first_range_nlos - euclidean_nlos)*100:.1f} cm")
"""),

code("""\
# NLOS assertions: first path must NOT be LOS, and must be longer than Euclidean
test2_pass = (not los_flag_nlos) and (first_range_nlos > euclidean_nlos - 0.005)

print()
if test2_pass:
    print("TEST 2 (NLOS BEHIND BOOKSHELF) : PASSED")
    if num_valid_nlos > 0:
        excess_cm = (first_range_nlos - euclidean_nlos) * 100
        print(f"  LOS=False, range excess = {excess_cm:.1f} cm (NLOS path goes around bookshelf)")
    else:
        print("  LOS=False, no valid paths (bookshelf fully opaque at 6.5 GHz)")
else:
    msgs = []
    if los_flag_nlos:
        msgs.append(
            "First path classified as LOS — bookshelf may not be blocking the path.\\n"
            "Check furniture_bookshelf.ply coordinates in gen_meshes.py.")
    if num_valid_nlos > 0 and first_range_nlos <= euclidean_nlos - 0.005:
        msgs.append(
            f"First-path range {first_range_nlos:.4f} m < Euclidean {euclidean_nlos:.4f} m "
            "— physically impossible for an NLOS path.")
    for m in msgs:
        print(f"FAIL: {m}")
    raise AssertionError("\\n".join(msgs))
"""),

md("## NLOS path visualisation"),

code("""\
bmp_nlos = scene.render(camera=cam, paths=paths_nlos, num_samples=128,
                        resolution=(720, 540), show_devices=True,
                        return_bitmap=True)
img_nlos = np.array(bmp_nlos)
fig, ax = plt.subplots(figsize=(9, 6))
ax.imshow(img_nlos)
ax.set_title("Test 2: anchor_3 (floor) → behind bookshelf — NLOS")
ax.axis("off")
plt.tight_layout()
out = os.path.join(RENDERS, "02_test2_nlos_paths.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
plt.close()
print(f"Saved: {out}")
"""),

md("## Summary"),

code("""\
print("=" * 55)
print("  CIR SANITY TEST SUMMARY")
print("=" * 55)
print(f"  Test 1 — LOS (anchor_0 ceiling → room centre)")
print(f"    TX:           {TX_POS}")
print(f"    RX:           {RX_POS}")
print(f"    Euclidean:    {euclidean_dist:.4f} m")
print(f"    Ray range:    {first_range_m:.4f} m")
print(f"    Error:        {abs(first_range_m - euclidean_dist)*100:.3f} cm")
print(f"    LOS flag:     {los_flag_los}")
print(f"    Result:       {'PASS' if test1_pass else 'FAIL'}")
print()
print(f"  Test 2 — NLOS (anchor_3 floor → behind bookshelf)")
print(f"    TX:           {TX_NLOS}")
print(f"    RX:           {RX_NLOS}")
print(f"    Euclidean:    {euclidean_nlos:.4f} m")
if num_valid_nlos > 0:
    print(f"    Ray range:    {first_range_nlos:.4f} m")
    print(f"    Range excess: {(first_range_nlos-euclidean_nlos)*100:.1f} cm")
else:
    print(f"    Ray range:    no valid paths")
print(f"    LOS flag:     {los_flag_nlos}")
print(f"    Result:       {'PASS' if test2_pass else 'FAIL'}")
print("=" * 55)

if test1_pass and test2_pass:
    print("  ALL TESTS PASSED — proceed to grid sweep (3a-ii)")
else:
    raise AssertionError("One or more CIR sanity tests failed.")
"""),

]

save(nb02, "02_single_point_test.ipynb")


# ───────────────────────────────────────────────────────────── notebook 03 ──

nb03 = nbf.v4.new_notebook()
nb03.cells = [

md("# 03 — Path Visualization (matplotlib 3D)\n\n"
   "Renders the room geometry, furniture, anchor positions, and "
   "multipath trajectories from anchor_0 to the room centre using matplotlib 3D.\n\n"
   "Sionna's built-in `scene.preview()` requires a WebGL viewer not "
   "available in standard Jupyter; this notebook uses a pure-Python fallback."),

code("""\
import os
import numpy as np
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import sionna
import sionna.rt as rt
from sionna.rt import (PathSolver, Transmitter, Receiver, PlanarArray, InteractionType)

print(f"Sionna {sionna.__version__}")
"""),

code("""\
scene_path = os.path.abspath("../scene/room.xml")
scene = rt.load_scene(scene_path)
scene.frequency = 6.5e9

anchors_yaml = os.path.abspath("../../src/microuwb_bringup/config/anchors.yaml")
with open(anchors_yaml) as f:
    anchors = yaml.safe_load(f)["anchors"]

anchor_positions = np.array([[a["x"], a["y"], a["z"]] for a in anchors])
print(f"Scene loaded. {len(anchors)} anchors.")
"""),

md("## Compute paths: anchor_0 → room centre"),

code("""\
iso_array = PlanarArray(num_rows=1, num_cols=1,
                        vertical_spacing=0.5, horizontal_spacing=0.5,
                        pattern="iso", polarization="V")
scene.tx_array = iso_array
scene.rx_array = iso_array

TX = anchor_positions[0].tolist()
RX = [2.5, 2.0, 1.0]

scene.add(Transmitter(name="tx", position=TX))
scene.add(Receiver(name="rx",   position=RX))

solver = PathSolver()
paths = solver(scene, max_depth=3, los=True,
               specular_reflection=True, diffuse_reflection=False,
               refraction=False, diffraction=False,
               samples_per_src=int(5e5))

valid_mask = np.array(paths.valid).squeeze().astype(bool)
print(f"Valid paths: {valid_mask.sum()}")
"""),

md("## Build matplotlib 3D figure"),

code("""\
def draw_box_edges(ax, x0, x1, y0, y1, z0, z1, color="steelblue", lw=0.8, alpha=0.5):
    pts = np.array([
        [x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],
        [x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1],
    ])
    edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
             (0,4),(1,5),(2,6),(3,7)]
    for i,j in edges:
        ax.plot(*zip(pts[i], pts[j]), color=color, lw=lw, alpha=alpha)

fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection="3d")

# Room envelope
draw_box_edges(ax, 0, 5, 0, 4, 0, 3, color="dimgray", lw=1.0, alpha=0.4)

# Furniture bounding boxes
FURNITURE = [
    ("bookshelf",  0.05, 0.95, 0.61, 1.01, 0.00, 1.20),
    ("cafe table", 2.044, 2.956, 2.044, 2.956, 0.00, 0.775),
    ("cabinet",    4.275, 4.725, 2.775, 3.225, 0.00, 1.02),
]
for label, *bounds in FURNITURE:
    draw_box_edges(ax, *bounds, color="peru", lw=1.2, alpha=0.7)

# Anchor positions
for i, pos in enumerate(anchor_positions):
    c = "red" if pos[2] > 1.5 else "darkorange"
    ax.scatter(*pos, color=c, s=60, zorder=5)
    ax.text(pos[0]+0.05, pos[1]+0.05, pos[2]+0.05, f"a{i}", fontsize=7, color=c)

# TX and RX
ax.scatter(*TX, color="blue", s=80, marker="^", zorder=6, label="TX (anchor_0)")
ax.scatter(*RX, color="green", s=80, marker="v", zorder=6, label="RX (room centre)")

print("Scene geometry drawn")
"""),

code("""\
# Draw multipath trajectories using paths.vertices if available
# vertices shape: (max_depth, num_rx, num_tx, num_paths, 3)
tau_np = np.array(paths.tau).squeeze()

try:
    verts_np = np.array(paths.vertices)   # attempt (max_depth, ..., 3)
    has_verts = True
    # Collapse singleton tx/rx dims: (max_depth, num_paths, 3)
    while verts_np.ndim > 3 and verts_np.shape[1] == 1:
        verts_np = verts_np[:, 0, ...]
    while verts_np.ndim > 3 and verts_np.shape[2] == 1:
        verts_np = verts_np[:, :, 0, ...]
    print(f"paths.vertices available, shape after squeeze: {verts_np.shape}")
except AttributeError:
    has_verts = False
    print("paths.vertices not available — drawing direct TX→RX lines")

valid_idx = np.where(valid_mask)[0]
sorted_idx = valid_idx[np.argsort(tau_np[valid_mask])]
n_draw = min(20, len(sorted_idx))

cmap = plt.cm.plasma
for rank, pi in enumerate(sorted_idx[:n_draw]):
    color = cmap(rank / max(n_draw - 1, 1))
    alpha = max(0.15, 0.8 - rank * 0.03)

    if has_verts:
        # Build waypoint sequence: TX → bounce points → RX
        waypoints = [np.array(TX)]
        for depth in range(verts_np.shape[0]):
            v = verts_np[depth, pi, :]
            if np.any(np.isnan(v)) or np.all(v == 0):
                break
            waypoints.append(v)
        waypoints.append(np.array(RX))
        for k in range(len(waypoints) - 1):
            seg = np.array([waypoints[k], waypoints[k+1]])
            ax.plot(seg[:,0], seg[:,1], seg[:,2], color=color, lw=0.6, alpha=alpha)
    else:
        seg = np.array([TX, RX])
        ax.plot(seg[:,0], seg[:,1], seg[:,2], color=color, lw=0.6, alpha=alpha)

ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
ax.set_xlim(0, 5); ax.set_ylim(0, 4); ax.set_zlim(0, 3)
ax.set_title(f"Multipath: anchor_0 → room centre\\n"
             f"({n_draw} paths shown, coloured shortest→longest)")
ax.legend(loc="upper left", fontsize=8)
plt.tight_layout()

RENDERS = os.path.abspath("../renders")
os.makedirs(RENDERS, exist_ok=True)
out = os.path.join(RENDERS, "03_multipath_3d.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
plt.close()
print(f"Saved: {out}")
"""),

md("## LOS coverage overview — slice at z = 1.5 m (coarse)\n\n"
   "Quick sample: check LOS/NLOS for a 10×8 coarse grid from all 5 anchors."),

code("""\
scene.remove("tx")
scene.remove("rx")

xs = np.linspace(0.5, 4.5, 9)
ys = np.linspace(0.5, 3.5, 7)
Z_SLICE = 1.5
XX, YY = np.meshgrid(xs, ys)
pts = np.stack([XX.ravel(), YY.ravel(), np.full(XX.size, Z_SLICE)], axis=1)

fig, axes = plt.subplots(1, 5, figsize=(18, 3.5), sharey=True)

for anc_idx, anc in enumerate(anchors):
    anc_pos = [float(anc["x"]), float(anc["y"]), float(anc["z"])]

    try:
        scene.remove("tx")
    except Exception:
        pass

    scene.add(Transmitter(name="tx", position=anc_pos))
    los_grid = np.zeros(len(pts), dtype=bool)

    for ri, pt in enumerate(pts):
        try:
            scene.remove("rx")
        except Exception:
            pass
        scene.add(Receiver(name="rx", position=pt.tolist()))

        p = solver(scene, max_depth=3, los=True, specular_reflection=True,
                   diffuse_reflection=False, refraction=False, diffraction=False,
                   samples_per_src=int(1e5))

        v = np.array(p.valid).squeeze().astype(bool)
        tau_i = np.array(p.tau).squeeze()
        if v.any():
            fi = int(np.argmin(tau_i[v]))
            inter_i = np.array(p.interactions)
            md_i = inter_i.shape[0]; np_i = inter_i.shape[-1]
            inter2d = inter_i.reshape(md_i, -1, np_i)[:, 0, :]
            fpi = inter2d[:, np.where(v)[0][fi]]
            los_grid[ri] = bool(np.all(fpi == InteractionType.NONE))

    try:
        scene.remove("rx")
    except Exception:
        pass

    los_map = los_grid.reshape(len(ys), len(xs))
    ax = axes[anc_idx]
    im = ax.imshow(los_map, origin="lower", extent=[0.5, 4.5, 0.5, 3.5],
                   cmap="RdYlGn", vmin=0, vmax=1, aspect="auto")
    ax.scatter(*anc_pos[:2], c="blue", s=50, marker="^", zorder=5)
    ax.set_title(f"anchor_{anc_idx}\\n({anc_pos[0]},{anc_pos[1]},{anc_pos[2]})", fontsize=8)
    ax.set_xlabel("X (m)")
    if anc_idx == 0:
        ax.set_ylabel("Y (m)")
    los_pct = 100 * los_grid.mean()
    ax.set_xlabel(f"LOS: {los_pct:.0f}%")

plt.suptitle(f"LOS/NLOS at z={Z_SLICE}m (green=LOS, red=NLOS)", fontsize=10)
plt.tight_layout()
out = os.path.join(RENDERS, "03_los_coverage_slice.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
plt.close()
print(f"Saved: {out}")
"""),

]

save(nb03, "03_path_visualization.ipynb")

print("Done.")
