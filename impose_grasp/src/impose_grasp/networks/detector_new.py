from impose_grasp.networks.darknet_detector_new import NewDarknetDetector
from impose_grasp.networks.pvn_detector_new import PvnDetector
from impose_grasp.models.cameras.base_camera import CamFrame

class PositionDetector():
    def __init__(self, target_obj_name) -> None:
        self.darknet_detector = NewDarknetDetector(target_obj_name)
        self.pvn_detector = PvnDetector(target_obj_name)

    def detect_object(self, frame: CamFrame):
        bbox = self.darknet_detector.inference(frame)
        affine_matrix = self.pvn_detector.pose_estimation(frame, bbox)
        return affine_matrix