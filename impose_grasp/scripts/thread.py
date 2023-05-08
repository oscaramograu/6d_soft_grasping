import cv2
import numpy as np
from scipy.spatial.transform.rotation import Rotation as R
from typing import Dict
import os
import time
from threading import Thread
from typing import List, Callable
import open3d as o3d

from include.app.app import App
from include.models.cameras.realsense_D415 import D415
# from include.models.cameras.base_camera import Camera, CamFrame
from include.models.dt_object import DTObject
from include.networks.pose_detection_network import PoseDetectionNetwork
from include.lib.geometry import invert_homogeneous
from include.lib.utils import time_stamp

class CameraThread:
    def __init__(self): #, cb_camera: Callable[[CamFrame], None] = None):
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


    def create_dtObject(self, obj_id):
        n_objs = sum([x.startswith(obj_id)
                     for x in self.dt_objectcs_list.keys()])
        unique_obj_name = f"{obj_id}__{n_objs:02}"
        meshpath = os.path.join("data", "models", obj_id, f"{obj_id}.ply")

        new_obj = DTObject(name=unique_obj_name,
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

    def get_dtObjDetections(self, frame):
        confidence_threshold = self.settings['confidence']
        darknet_frame, detections, _ = self.obj_detector.image_detection(
            frame.rgb.copy(), confidence_threshold, None)
        # print(detections)
        detections = [x for x in detections if x[0] == self.dt_obj_type]
        return detections
    
    def get_bboxFromPredictedPVN(self, frame):
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
    
    def main(self):
        prev_pose = None
        prev_omega = None
        omega = None
        v = None
        prev_v = None
        frame = None
        prior_obj_type = None
        prior_cam = None
        detected = False

        while not self.stop_flag:
            self.manage_fps()

            frame = self.cam.grab_frame()
            if frame is None:
                continue

            # Image (2D object detection/prediction)
            bbox = None
            detections = self.get_dtObjDetections(frame)
            if len(detections) > 0:
                bbox_xywh = detections[-1][2]
                confidence = detections[-1][1]
                bbox = self.obj_detector.yolo_bbox_2_original(
                    bbox_xywh, frame.rgb.shape[:2])

            if detected:
                # if previously detected calculate the bounding box from pvn prediction
                # project the bounding box of the object into the image
                bbox = self.get_bboxFromPredictedPVN(frame)

            # 6D pose estimation of selected object
            detected = False
            affine_matrix = None
            if bbox is not None and self.pose_detector is not None:
                try:
                    detected, affine_matrix = self.pose_detector.inference(bbox, frame.rgb.copy(
                    ), frame.depth, frame.intrinsic, use_icp=self.use_icp)  # affine_matrix.shape = 3x4 !
                except Exception as e:
                    print(f"[{time_stamp()}] pose detection failed with ", e)

            if detected:
                current_pose = frame.extrinsic @ affine_matrix
                if prev_pose is not None:
                    v = np.linalg.norm(
                        current_pose[:3, 3] - prev_pose[:3, 3]) / dt  # m / sec
                    omega = R.from_matrix(
                        current_pose[:3, :3]) * R.from_matrix(prev_pose[:3, :3].T)
                    omega = omega.magnitude() / dt  # rad/sec
                    #print(f"Object [v|omega]: {v:.4f}, {omega/np.pi*180:.2f}")

                    if prev_v is not None:
                        v_dot = (v - prev_v) / dt
                        omega_dot = (omega - prev_omega) / dt
                        #print(f"Object [v_dot|omega_dot]: {v_dot:.4f}, {omega_dot/np.pi*180:.2f}")

                    vel_limit = v > 0.12
                    omega_limit = omega > np.pi*0.8
                    if vel_limit or omega_limit:
                        detected = False
                        reason = "velocity" if vel_limit else "omega"
                        print(
                            f"[{time_stamp()}] Using darknet because of {reason}")

                prev_pose = current_pose
                if prev_pose is not None:
                    prev_omega = omega
                    prev_v = v
                else:
                    prev_omega = None
                    prev_v = None
                dt_obj.show()
                dt_obj.transform(current_pose)

            else:
                dt_obj.hide()
                prev_pose = None
                prev_omega = None
                prev_v = None

            # update digital twin obstruction
            self.obstruction_pcd.update_from_scene(
                frame, dt_obj, affine_matrix)

            # annotate
            annotated = frame.rgb.copy()
            if self.draw_object:
                if bbox is not None:
                    annotated = cv2.rectangle(annotated, (int(bbox[0]), int(
                        bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 0), thickness=4)
                if dt_obj.active:
                    annotated = poseDetection.draw_obj(
                        annotated, affine_matrix,  frame.intrinsic)

            cv2.putText(annotated, f"{1/dt:4.1f} FPS", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), thickness=3)

            anno_frame = CamFrame(rgb=annotated, depth=frame.depth.copy(
            ), intrinsic=frame.intrinsic, extrinsic=frame.extrinsic, rgb2=darknet_frame)

            if self.cb_camera is not None:
                self.cb_camera(anno_frame)