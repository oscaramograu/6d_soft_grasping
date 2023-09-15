import numpy as np
from math import pi

class VectorCreator:
    vector: np.ndarray

    def create_vector_from_2_pts(self, init_pt: np.ndarray, end_pt:np.ndarray):
        self.vector = end_pt - init_pt
    def set_vector(self, vector):
        self.vector = vector

class VectorOperator:
    def compute_angle(self, init_vect, end_vect):
        dot_prod = np.dot(init_vect, end_vect)
        norm_i, norm_e = np.linalg.norm(init_vect), np.linalg.norm(end_vect)
        angle = np.arccos(dot_prod/(norm_i*norm_e))
        return angle
    
    def compute_sign(self, init_vect, end_vect):
        cross_prod = np.cross(init_vect, end_vect)
        norm_i, norm_e = np.linalg.norm(init_vect), np.linalg.norm(end_vect)
        sign = np.arcsin(cross_prod/(norm_i*norm_e))
        return sign
    
    def perpendicular_plane(self, vector):
        """
        Given a 3D vector, returns the equation of the perpendicular plane in the form
        Ax + By + Cz + D = 0, where (A, B, C) is the normal vector and D is the constant.
        """
        if len(vector) != 3:
            raise ValueError("Input vector must be a 3D vector")

        A, B, C = vector

        D = -A * 0 - B * 0 - C * 0

        return A, B, C, D
    
    def random_vector_on_plane(self, plane_equation):
        """
        Given the equation of a plane in the form Ax + By + Cz + D = 0,
        generates a random vector on the plane.
        """
        A, B, C, D = plane_equation

        x = np.random.uniform(-10, 10)
        y = np.random.uniform(-10, 10)

        z = (-A * x - B * y - D) / C

        random_vector = np.array([x, y, z])

        return random_vector
    
    def project_vector_onto_plane(self, vector, plane_equation):
        """
        Given a 3D vector and the equation of a plane in the form Ax + By + Cz + D = 0,
        returns the projected vector of the input vector onto the plane.
        """
        A, B, C, D = plane_equation

        # Calculate the scalar projection of the input vector onto the plane's normal vector
        scalar_projection = np.dot(vector, [A, B, C]) / np.linalg.norm([A, B, C]) ** 2

        # Calculate the projected vector
        projected_vector = vector - scalar_projection * np.array([A, B, C])

        return projected_vector

    def rotate_vector_around_plane(self, vector, plane_equation, angle_degrees):
        """
        Given a 3D vector, the equation of a plane in the form Ax + By + Cz + D = 0,
        and an angle in degrees, rotates the input vector around the plane by the specified angle.
        """
        A, B, C, _ = plane_equation

        # Convert the angle from degrees to radians
        angle_radians = np.radians(angle_degrees)

        # Create the rotation matrix
        cos_theta = np.cos(angle_radians)
        sin_theta = np.sin(angle_radians)
        rotation_matrix = np.array([[cos_theta + A**2 * (1 - cos_theta), A * B * (1 - cos_theta) - C * sin_theta, A * C * (1 - cos_theta) + B * sin_theta],
                                    [B * A * (1 - cos_theta) + C * sin_theta, cos_theta + B**2 * (1 - cos_theta), B * C * (1 - cos_theta) - A * sin_theta],
                                    [C * A * (1 - cos_theta) - B * sin_theta, C * B * (1 - cos_theta) + A * sin_theta, cos_theta + C**2 * (1 - cos_theta)]])

        # Rotate the vector using the rotation matrix
        rotated_vector = np.dot(rotation_matrix, vector)

        return rotated_vector
    
    def intersection_vector(self, plane1_equation, plane2_equation):
        """
        Given the equations of two planes in the form Ax + By + Cz + D = 0,
        returns the intersection vector between the two planes.
        """
        A1, B1, C1, _ = plane1_equation
        A2, B2, C2, _ = plane2_equation

        # Calculate the normal vectors of both planes
        normal_vector1 = np.array([A1, B1, C1])
        normal_vector2 = np.array([A2, B2, C2])

        # Calculate the intersection vector as the cross product of the normal vectors
        intersection_vector = np.cross(normal_vector1, normal_vector2)

        return intersection_vector
    
    def rotate_Rt_on_z(self, array: np.ndarray, angle: float):
        new_arr = array.copy()
        angle_rad = angle*pi/180

        rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                                    [np.sin(angle_rad), np.cos(angle_rad), 0],
                                    [0, 0, 1]])
        new_arr[:3, :3] = np.dot(new_arr[:3, :3], rotation_matrix)
        return new_arr