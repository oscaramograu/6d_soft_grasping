import cv2
import numpy as np
import os
import time
from typing import Callable

from include.app.app import App

from include.models.cameras.realsense_D415 import D415
from include.models.cameras.base_camera import CamFrame
from include.models.dt_object import DTObject
from include.models.obstruction_pcd import ObstructionPcd

from include.networks.pose_detection_network import PoseDetectionNetwork
from include.lib.geometry import invert_homogeneous
from include.lib.utils import time_stamp

class CameraThread:
    def __init__(self, cb_camera: Callable[[CamFrame], None] = None): #, cb_camera: Callable[[CamFrame], None] = None):
            # flags
        self.stop_flag = False
        self.settings = App().settings['Object Detection']  # relevant settings
        self.t_last_cb_camera = None

        self.cam = D415(name="realsense_D415")

        self.dt_obj_type = "cpsduck"
        self.dt_obj = self.create_dtObject(self.dt_obj_type)

        self.obj_detector = self.create_objDetector()

        n_points = 512
        self.pose_detector = PoseDetectionNetwork(self.obj_type , n_points)

        self.obstruction_pcd = ObstructionPcd()

        self.draw_object = False
        self.darknet_frame = None
        self.dt = None
        self.cb_camera = cb_camera

    def create_dtObject(self, obj_id):
        meshpath = os.path.join("data", "models", obj_id, f"{obj_id}.ply")

        new_obj = DTObject(name=obj_id,
                           meshpath=meshpath, show_axes=True)
        return new_obj
    
    def create_objDetector(self):
        from include.networks.pvn.lib.main_darknet import MainDarknet 

        objectDetector = MainDarknet()
        obj_data = os.path.join("data", "weights", "darknet", "cps")
        objectDetector.cfg_file = os.path.join(obj_data, "yolo.cfg")
        objectDetector.data_file = os.path.join(obj_data, "obj.data")
        objectDetector.params.monitor_params.weights_path = os.path.join(obj_data, "yolo.weights")
        objectDetector.initial_trainer_and_model()

        return objectDetector
    
    def manage_fps(self):
        t_last_cb_camera = self.t_last_cb_camera
        dt = time.perf_counter() - t_last_cb_camera

        if dt < 1/self.settings['FPS']:
            # limit to 30 FPS
            time.sleep(1/self.settings['FPS'] - dt)
            dt = time.perf_counter() - t_last_cb_camera

        self.t_last_cb_camera = time.perf_counter()

        self.dt = dt

    def get_dtObjDetections_and_set_darknetFrame(self, frame: CamFrame):
        confidence_threshold = self.settings['confidence']
        self.darknet_frame, detections, _ = self.obj_detector.image_detection(
            frame.rgb.copy(), confidence_threshold, None)
        # print(detections)
        detections = [x for x in detections if x[0] == self.dt_obj_type]
        return detections
    
    def get_bboxOfObjInFrame(self, frame: CamFrame, detected):
        bbox = None
        detections = self.get_dtObjDetections_and_set_darknetFrame(frame)
        if len(detections) > 0:
            bbox_xywh = detections[-1][2]
            confidence = detections[-1][1]
            bbox = self.obj_detector.yolo_bbox_2_original(
                bbox_xywh, frame.rgb.shape[:2])

        if detected:
            # if previously detected calculate the bounding box from pvn prediction
            # project the bounding box of the object into the image
            bbox = self.get_bboxProjected(frame)

        return bbox
    
    def get_bboxProjected(self, frame: CamFrame):
        affine_matrix = invert_homogeneous(
            frame.extrinsic) @ self.dt_obj.pose
        pvn_bbox = self.dt_obj.bounding_box @ affine_matrix[:3, :3].T
        pvn_bbox += affine_matrix[:3, 3].T
        bbox_in_img = pvn_bbox[:, :3]
        bbox_in_img /= bbox_in_img[:, 2:3]
        bbox_in_img[:, 2] = 1.
        bbox_in_img = bbox_in_img @ frame.intrinsic.T

        # get image bounding box
        x_min, y_min = np.min(bbox_in_img[:, :2], axis=0)
        x_max, y_max = np.max(bbox_in_img[:, :2], axis=0)
        pvn_bbox = np.array([x_min, y_min, x_max, y_max])
        pvn_bbox[::2] = np.clip(pvn_bbox[::2], 0, frame.rgb.shape[1])
        pvn_bbox[1::2] = np.clip(pvn_bbox[1::2], 0, frame.rgb.shape[0])

        return pvn_bbox
    
    def detectAndGet_AffineMatrix(self,frame: CamFrame, bbox):
        affine_matrix = None
        if bbox is not None and self.pose_detector is not None:
            try:
                detected, affine_matrix = self.pose_detector.inference(bbox, frame.rgb.copy(
                ), frame.depth, frame.intrinsic, use_icp=self.use_icp)  # affine_matrix.shape = 3x4 !
            except Exception as e:
                print(f"[{time_stamp()}] pose detection failed with ", e)
        return detected, affine_matrix
    
    def annotate_frameWithBbox(self, frame: CamFrame, bbox, affine_matrix):
        annotated = frame.rgb.copy()
        if self.draw_object:
            if bbox is not None:
                annotated = cv2.rectangle(annotated, (int(bbox[0]), int(
                    bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 0), thickness=4)
            if self.dt_obj.active:
                annotated = self.pose_detector.draw_obj(
                    annotated, affine_matrix,  frame.intrinsic)

        cv2.putText(annotated, f"{1/self.dt:4.1f} FPS", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), thickness=3)

        anno_frame = CamFrame(rgb=annotated, depth=frame.depth.copy(), 
            intrinsic=frame.intrinsic, extrinsic=frame.extrinsic, rgb2=self.darknet_frame)

        return anno_frame

    def main(self):
        frame = None
        detected = False

        while not self.stop_flag:
            self.manage_fps()

            frame = self.cam.grab_frame()
            if frame is None:
                continue

            # Image (2D object detection/prediction)
            bbox = self.get_bboxOfObjInFrame(self, frame, detected)

            # 6D pose estimation of selected object
            detected = False
            detected, affine_matrix = self.detectAndGet_AffineMatrix(frame, bbox)

            if detected:
                current_pose = frame.extrinsic @ affine_matrix

                self.dt_obj.show()
                self.dt_obj.transform(current_pose)

            else:
                self.dt_obj.hide()

            # update digital twin obstruction
            self.obstruction_pcd.update_from_scene(
                frame, self.dt_obj, affine_matrix)

            # annotate
            anno_frame = self.annotate_frameWithBbox(frame, bbox, affine_matrix)

            if self.cb_camera is not None:
                self.cb_camera(anno_frame)