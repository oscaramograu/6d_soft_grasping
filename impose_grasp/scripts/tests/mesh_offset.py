import trimesh
import numpy as np
import os
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP

mesh_pth = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "models", "qb_hand_col.stl")
mesh_pinch_pth = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "models", "qb_hand_col_pinch.stl")

# Load the stl file
mesh = trimesh.load_mesh(mesh_pth)

# Define the offset vector (in this case, 2 cm in the -Z direction)
offset = [-0.015, 0, 0.01]  # Offset is specified in meters

# Apply the offset to all vertices
mesh.vertices += offset

# Define the rotation angle in degrees
rotation_angle = -30  # Rotate by 30 degrees around Z-axis

# Convert rotation angle to radians
rotation_angle_rad = np.radians(rotation_angle)

# Define the rotation matrix for Z-axis rotation
rotation_matrix = np.array([[np.cos(rotation_angle_rad), -np.sin(rotation_angle_rad), 0],
                            [np.sin(rotation_angle_rad), np.cos(rotation_angle_rad), 0],
                            [0, 0, 1]])

# Apply the rotation to all vertices
mesh.vertices = np.dot(mesh.vertices, rotation_matrix.T)
# Save the offset mesh back to the stl file
mesh.export(mesh_pinch_pth)