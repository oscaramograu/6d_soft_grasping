from impose_grasp.networks.darknet import DarknetDetector
from impose_grasp.networks.pvn import PvnDetector

from impose_grasp.models.cameras.base_camera import CamFrame

class Detector():
    def __init__(self, target_obj_name) -> None:
        self.darknet_detector = DarknetDetector(target_obj_name)

        n_points = 512
        self.pvn_detector = PvnDetector(target_obj_name, n_points)

        self.frame: CamFrame
        self.bbox = None
        self.affine_matrix = None
        self.detected = None

    def set_frame(self, frame: CamFrame):
        self.frame = frame

    def compute_bbox(self):
        self.bbox = self.darknet_detector.inference(self.frame)
    
    def compute_affine(self):        
    # 6D pose estimation of selected object        
        _use_icp = False
        if self.bbox is not None:
            detected, affine_matrix = self.pvn_detector.inference(self.bbox, self.frame.rgb.copy(
                ), self.frame.depth, self.frame.intrinsic, use_icp=_use_icp)  # affine_matrix.shape = 3x4 !

            self.affine_matrix = affine_matrix
            self.detected = detected
        else:
            self.affine_matrix = None
            self.detected = None
    
    def get_affine(self):
        if self.detected:
            return self.affine_matrix
        else:
            print("No object was detected")
            return None
