import trimesh
import numpy as np
import os
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP

mesh_pth = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "models", "qb_hand_col_fingers.stl")
mesh_pinch_pth = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "models", "qb_hand_col_pinch_fingers.stl")
mesh_pinch_plypth = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "models", "qb_hand_col_pinch_fingers.ply")

# Load the stl file
mesh = trimesh.load_mesh(mesh_pth)

# Define the offset vector (in this case, 2 cm in the -Z direction)
offset = [-0.005, -0.017, 0.01]  # Offset is specified in meters
rotation_angle_Z = -30  # Rotate by 30 degrees around Z-axis
rotation_angle_X = 0  # Rotate by 30 degrees around Z-axis

# Apply the offset to all vertices
mesh.vertices += offset

# Convert rotation angle to radians
rotation_angle_rad_Z = np.radians(rotation_angle_Z)

# Define the rotation matrix for Z-axis rotation
rotation_Z = np.array([[np.cos(rotation_angle_rad_Z), -np.sin(rotation_angle_rad_Z), 0],
                            [np.sin(rotation_angle_rad_Z), np.cos(rotation_angle_rad_Z), 0],
                            [0, 0, 1]])

# Apply the rotation to all vertices
mesh.vertices = np.dot(mesh.vertices, rotation_Z.T)

# Convert rotation angle to radians
rotation_angle_rad_X = np.radians(rotation_angle_X)

# Define the rotation matrix for X-axis rotation
rotation_X = np.array([[1, 0, 0],
    [0, np.cos(rotation_angle_rad_X), -np.sin(rotation_angle_rad_X)],
    [0, np.sin(rotation_angle_rad_X), np.cos(rotation_angle_rad_X)]])

# Apply the rotation to all vertices
mesh.vertices = np.dot(mesh.vertices, rotation_X.T)

# Save the offset mesh back to the stl file
mesh.export(mesh_pinch_pth)
# Save the offset mesh back to the stl file
mesh.export(mesh_pinch_plypth)