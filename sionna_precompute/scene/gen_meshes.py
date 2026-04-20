"""
Generate ASCII PLY mesh files for a 5×4×3m room + furniture bounding boxes.

Room spans (0,0,0) → (5,4,3). Origin at SW floor corner.
Surface normals point INTO the room (required for correct ray tracing).
Winding verified by cross-product: (v1-v0) × (v2-v0) = outward normal.

Furniture is modelled as solid AABB boxes. Normals point AWAY from the solid
(into the air), matching Sionna's convention for all surfaces.
"""
import os

MESH_DIR = os.path.join(os.path.dirname(__file__), "meshes")
os.makedirs(MESH_DIR, exist_ok=True)


def write_ply(name: str, vertices: list, faces: list) -> None:
    path = os.path.join(MESH_DIR, name)
    with open(path, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(vertices)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")
        for v in vertices:
            f.write(f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for face in faces:
            f.write(f"{len(face)} " + " ".join(str(i) for i in face) + "\n")
    print(f"  wrote {path}")


def gen_box_ply(name: str, x0: float, x1: float,
                y0: float, y1: float, z0: float, z1: float) -> None:
    """
    Write a 6-face triangulated box with outward-facing normals.
    Normal verification (n = (v1-v0) × (v2-v0)):
      bottom [0,3,2]: (0,dy,0)×(dx,dy,0) → (0,0,-dx·dy)  → -Z ✓
      top    [4,5,6]: (dx,0,0)×(dx,dy,0) → (0,0,+dx·dy)  → +Z ✓
      south  [0,1,5]: (dx,0,0)×(dx,0,dz) → (0,-dx·dz,0)  → -Y ✓
      north  [2,3,7]: (-dx,0,0)×(-dx,0,dz) → (0,+dx·dz,0) → +Y ✓
      west   [0,4,7]: (0,0,dz)×(0,dy,dz) → (-dz·dy,0,0)  → -X ✓
      east   [1,2,6]: (0,dy,0)×(0,dy,dz) → (+dy·dz,0,0)  → +X ✓
    """
    verts = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),  # 0-3 bottom
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),  # 4-7 top
    ]
    faces = [
        [0, 3, 2], [0, 2, 1],  # bottom  (-Z)
        [4, 5, 6], [4, 6, 7],  # top     (+Z)
        [0, 1, 5], [0, 5, 4],  # south   (-Y)
        [2, 3, 7], [2, 7, 6],  # north   (+Y)
        [0, 4, 7], [0, 7, 3],  # west    (-X)
        [1, 2, 6], [1, 6, 5],  # east    (+X)
    ]
    write_ply(name, verts, faces)


# ---------------------------------------------------------------------------
# Room surfaces  (normals face INTO the room)

# Floor  (z=0)  normal = +Z  (CCW from above)
write_ply("floor.ply",
    vertices=[(0,0,0), (5,0,0), (5,4,0), (0,4,0)],
    faces=[[0,1,2], [0,2,3]])

# Ceiling (z=3)  normal = -Z  (CW from above)
write_ply("ceiling.ply",
    vertices=[(0,0,3), (0,4,3), (5,4,3), (5,0,3)],
    faces=[[0,1,2], [0,2,3]])

# South wall (y=0)  normal = +Y
write_ply("wall_south.ply",
    vertices=[(0,0,0), (0,0,3), (5,0,3), (5,0,0)],
    faces=[[0,1,2], [0,2,3]])

# North wall (y=4)  normal = -Y
write_ply("wall_north.ply",
    vertices=[(0,4,0), (5,4,0), (5,4,3), (0,4,3)],
    faces=[[0,1,2], [0,2,3]])

# West wall  (x=0)  normal = +X
write_ply("wall_west.ply",
    vertices=[(0,0,0), (0,4,0), (0,4,3), (0,0,3)],
    faces=[[0,1,2], [0,2,3]])

# East wall  (x=5)  normal = -X
write_ply("wall_east.ply",
    vertices=[(5,0,0), (5,0,3), (5,4,3), (5,4,0)],
    faces=[[0,1,2], [0,2,3]])

# ---------------------------------------------------------------------------
# Furniture bounding boxes  (normals face away from solid, into air)
#
# Positions derived from room.world.xacro model poses and Gazebo SDF geometry:
#   bookshelf  pose(0.5, 1.0, 0): local AABB x[-0.45,0.45] y[-0.395,0.01] z[0,1.2]
#   cafe_table pose(2.5, 2.5, 0): top at z=0.775, footprint 0.913m sq
#   cabinet    pose(4.5, 3.0, 0) rot 90°: 0.45×0.45×1.02m

gen_box_ply("furniture_bookshelf.ply",
    x0=0.05,  x1=0.95,
    y0=0.61,  y1=1.01,
    z0=0.00,  z1=1.20)

gen_box_ply("furniture_cafe_table.ply",
    x0=2.044, x1=2.956,
    y0=2.044, y1=2.956,
    z0=0.00,  z1=0.775)

gen_box_ply("furniture_cabinet.ply",
    x0=4.275, x1=4.725,
    y0=2.775, y1=3.225,
    z0=0.00,  z1=1.02)

print("All mesh files generated.")
