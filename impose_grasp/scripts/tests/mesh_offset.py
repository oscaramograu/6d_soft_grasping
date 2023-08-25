import trimesh
import os
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP

mesh_pth = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "models", "qb_hand_col3.stl")

# Load the stl file
mesh = trimesh.load_mesh(mesh_pth)

# Define the offset vector (in this case, 2 cm in the -Z direction)
offset = [0, 0, +0.04]  # Offset is specified in meters

# Apply the offset to all vertices
mesh.vertices += offset

# Save the offset mesh back to the stl file
mesh.export(mesh_pth)