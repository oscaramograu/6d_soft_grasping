import trimesh
import os
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP

mesh_pth = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "models", "qb_hand_col_fingers.stl")

# Load the PLY file
mesh = trimesh.load_mesh(mesh_pth)

# Scale down the vertices by a factor of 0.001 (1/1000)
mesh.vertices *= 0.001

# Save the scaled mesh back to the PLY file
mesh.export(mesh_pth)
