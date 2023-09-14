import numpy as np

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