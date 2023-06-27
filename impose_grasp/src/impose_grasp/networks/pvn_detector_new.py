from impose_grasp.networks.pose_detection_network import PoseDetectionNetwork
from impose_grasp.models.cameras.base_camera import CamFrame
import numpy as np

class PvnDetector():
    def __init__(self, dt_obj_type: str) -> None:
        n_points = 512
        self.pose_detector = PoseDetectionNetwork(dt_obj_type, n_points)

        self._freeze_object: bool
        self._use_icp = False

    def pose_estimation(self, frame: CamFrame, bbox):
        # 6D pose estimation of selected object        
        if bbox is not None and self.pose_detector is not None:
            try:
                detected, affine_matrix = self.pose_detector.inference(bbox, frame.rgb.copy(
                ), frame.depth, frame.intrinsic, use_icp=self._use_icp)  # affine_matrix.shape = 3x4 !
            except Exception as e:
                print(" pose detection failed with ", e)
        
        return affine_matrix
