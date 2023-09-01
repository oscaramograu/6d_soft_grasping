import os
import sys

from impose_grasp.lib.utils import SIX_IMPOSE_PATH, PATH_TO_IMPOSE_GRASP
sys.path.append(SIX_IMPOSE_PATH)

from lib.main_darknet import MainDarknet
from impose_grasp.models.cameras.base_camera import CamFrame


class DarknetDetector():
    def __init__(self, obj_name) -> None:
        self.confidence = 0.05  # settings

        self._dt_obj_type = obj_name
        self._obj_detector = self._create_obj_detector()

    def inference(self, frame: CamFrame):
        """
            Use darknet to compute a bounding box of the target
            objects detected in a given image frame

            I nothing was detected returns None
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
        """
            Initializes a darknet object using the following files:
            - weights: yolo.weights (weights of the pre-trained model)
            - cfg: yolo.cfg (network parameters)
            - data: obj.data (list of target objects)
        """
        objectDetector = MainDarknet()
        obj_data = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "darknet")
        weights_path =  os.path.join(obj_data, "yolo.weights")

        objectDetector.cfg_file = os.path.join(obj_data, "yolo.cfg")
        objectDetector.data_file = os.path.join(obj_data, "obj.data")
        objectDetector.params.monitor_params.weights_path = weights_path
        objectDetector.initial_trainer_and_model()

        return objectDetector