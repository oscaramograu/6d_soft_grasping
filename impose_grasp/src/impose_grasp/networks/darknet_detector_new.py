import os
import sys
import numpy as np
from typing import Dict

from impose_grasp.lib.utils import SIX_IMPOSE_PATH, PATH_TO_IMPOSE_GRASP
sys.path.append(SIX_IMPOSE_PATH)

from lib.main_darknet import MainDarknet, MainDarknetParams
from impose_grasp.models.cameras.base_camera import CamFrame
from impose_grasp.lib.geometry import invert_homogeneous
from impose_grasp.models.dt_object import DTObject

class NewDarknetDetector():
    def __init__(self, obj_name) -> None:
        self.confidence = 0.05  # relevant settings

        self._dt_obj_type = obj_name
        # self._dt_object = self._create_object()

        self._obj_detector = self._create_obj_detector()

        self._detected: bool

    # def _compute_bounding_box(self, frame: CamFrame):
    #     self._camera_frame = frame
    #     if self._detected:
    #         bbox = self._pvn_projection()
    #     else:
    #         bbox = self._darknet_inference()
    #     return bbox

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
            pd = PositionDetector("cpsduck")

    #     """
    #       If previously detected calculate the bounding box from pvn prediction
    #       project the bounding box of the object into the image
    #     """
    #     frame = self._camera_frame

    #     affine_matrix = invert_homogeneous(
    #         frame.extrinsic) @ self._dt_object.pose
    #     pvn_bbox = self._dt_object.bounding_box @ affine_matrix[:3, :3].T
    #     pvn_bbox += affine_matrix[:3, 3].T
    #     bbox_in_img = pvn_bbox[:, :3]
    #     bbox_in_img /= bbox_in_img[:, 2:3]
    #     bbox_in_img[:, 2] = 1.
    #     bbox_in_img = bbox_in_img @ frame.intrinsic.T

    #     # get image bounding box
    #     x_min, y_min = np.min(bbox_in_img[:, :2], axis=0)
    #     x_max, y_max = np.max(bbox_in_img[:, :2], axis=0)
    #     pvn_bbox = np.array([x_min, y_min, x_max, y_max])
    #     pvn_bbox[::2] = np.clip(pvn_bbox[::2], 0, frame.rgb.shape[1])
    #     pvn_bbox[1::2] = np.clip(pvn_bbox[1::2], 0, frame.rgb.shape[0])
    #     bbox = pvn_bbox

    #     return bbox

    # def _create_object(self):
    #     obj_id = self._dt_obj_type
    #     meshpath = os.path.join("data", "models", obj_id, f"{obj_id}.ply")
    #     new_obj = DTObject(name=obj_id,
    #                        meshpath=meshpath, show_axes=True)
    #     return new_obj
    
    def _create_obj_detector(self):
        # params = MainDarknetParams()
        # params.dataset_params.train_batch_size = 1
        # params.dataset_params.val_batch_size = 1
        # params.dataset_params.data_name = 1
        # params.dataset_params.cls_type = 1
        # params.dataset_params.use_preprocessed = 1
        # params.dataset_params.size_all = 1
        # params.dataset_params.train_size = 1
        # params.dataset_params.augment_per_image = 1
        
        objectDetector = MainDarknet()
        obj_data = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "weights", "darknet", "cps")
        weights_path =  os.path.join(obj_data, "yolo.weights")
        objectDetector.cfg_file = os.path.join(obj_data, "yolo.cfg")
        objectDetector.data_file = os.path.join(obj_data, "obj.data")
        objectDetector.params.monitor_params.weights_path = weights_path
        objectDetector.initial_trainer_and_model()
        return objectDetector