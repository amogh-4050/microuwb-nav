"""
Generate ASCII PLY mesh files for a 5×4×3m room.

Room spans (0,0,0) → (5,4,3). Origin at SW floor corner.
Surface normals point INTO the room (required for correct ray tracing).
Winding verified by cross-product: (v1-v0) × (v2-v0) = outward normal.
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


# ---------------------------------------------------------------------------
# Floor  (z=0)  normal = +Z  (CCW from above)
# Vertices: SW, SE, NE, NW
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

print("All mesh files generated.")
