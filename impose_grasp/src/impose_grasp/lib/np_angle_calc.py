import numpy as np
from math import pi

def rotate_Rt_on_x(array: np.ndarray, angle: float):
    new_arr = array.copy()
    angle_rad = angle*pi/180

    rotation_matrix = np.array([[1, 0, 0],
                                 [0, np.cos(angle_rad), -np.sin(angle_rad)],
                                 [0, np.sin(angle_rad), np.cos(angle_rad)]])
    new_arr[:3, :3] = np.dot(new_arr[:3, :3], rotation_matrix)
    return new_arr

def rotate_Rt_on_y(array: np.ndarray, angle: float):
    new_arr = array.copy()
    angle_rad = angle*pi/180

    rotation_matrix = np.array([[np.cos(angle_rad), 0, np.sin(angle_rad)],
                                 [0, 1, 0],
                                 [-np.sin(angle_rad), 0, np.cos(angle_rad)]])
    new_arr[:3, :3] = np.dot(new_arr[:3, :3], rotation_matrix)
    return new_arr

def rotate_Rt_on_z(array: np.ndarray, angle: float):
    new_arr = array.copy()
    angle_rad = angle*pi/180

    rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                                 [np.sin(angle_rad), np.cos(angle_rad), 0],
                                 [0, 0, 1]])
    new_arr[:3, :3] = np.dot(new_arr[:3, :3], rotation_matrix)
    return new_arr

def rotate_x(arr, angle_degrees):
    angle_rad = np.radians(angle_degrees)
    rotation_matrix = np.array([[1, 0, 0],
                                 [0, np.cos(angle_rad), -np.sin(angle_rad)],
                                 [0, np.sin(angle_rad), np.cos(angle_rad)]])
    rotated_array = np.dot(arr, rotation_matrix)
    return rotated_array

def rotate_y(arr, angle_degrees):
    angle_rad = np.radians(angle_degrees)
    rotation_matrix = np.array([[np.cos(angle_rad), 0, np.sin(angle_rad)],
                                 [0, 1, 0],
                                 [-np.sin(angle_rad), 0, np.cos(angle_rad)]])
    rotated_array = np.dot(arr, rotation_matrix)
    return rotated_array

def rotate_z(arr, angle_degrees):
    angle_rad = np.radians(angle_degrees)
    rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                                 [np.sin(angle_rad), np.cos(angle_rad), 0],
                                 [0, 0, 1]])
    rotated_array = np.dot(arr, rotation_matrix)
    return rotated_array
