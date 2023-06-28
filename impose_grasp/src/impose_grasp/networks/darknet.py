import os
import sys

from impose_grasp.lib.utils import SIX_IMPOSE_PATH, PATH_TO_IMPOSE_GRASP
sys.path.append(SIX_IMPOSE_PATH)

from lib.main_darknet import MainDarknet
from impose_grasp.models.cameras.base_camera import CamFrame


class DarknetDetector():
    def __init__(self, obj_name) -> None:
        self.confidence = 0.05  # relevant settings

        self._dt_obj_type = obj_name
        self._obj_detector = self._create_obj_detector()

    def inference(self, frame: CamFrame):
        """
          If not previously detected calculate the bounding box from 
          darknet inference
        """
        confidence_threshold = self.confidence
        darknet_frame, detections, _ = self._obj_detector.image_detection(
            frame.rgb.copy(), confidence_threshold, None)

        detections = [x for x in detections if x[0] == self._dt_obj_type]
        if len(detections) > 0:
            bbox_xywh = detections[-1][2]
            confidence = detections[-1][1]
            bbox = self._obj_detector.yolo_bbox_2_original(
                bbox_xywh, frame.rgb.shape[:2])
            return bbox
        else:
            return None
    
    def _create_obj_detector(self):
        objectDetector = MainDarknet()
        obj_data = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "darknet")
        weights_path =  os.path.join(obj_data, "yolo.weights")

        objectDetector.cfg_file = os.path.join(obj_data, "yolo.cfg")
        objectDetector.data_file = os.path.join(obj_data, "obj.data")
        objectDetector.params.monitor_params.weights_path = weights_path
        objectDetector.initial_trainer_and_model()

        return objectDetector