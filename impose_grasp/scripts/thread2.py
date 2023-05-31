import cv2
import time
from typing import Callable
from threading import Thread
from impose_grasp.app.app import App
import os

from impose_grasp.models.cameras.realsense_D415 import D415
from impose_grasp.models.cameras.base_camera import CamFrame
from impose_grasp.models.targetmodel import TargetModel

from impose_grasp.lib.utils import load_mesh
from impose_grasp.networks.detector import PositionDetector

class CameraThread:
    BASE_PATH = os.path.join(os.getcwd() ,'impose_grasp','data', 'models', 'fanuc_crx_10ial')
    SIX_IMPOSE_PATH = os.path.join("home", "oscar", "Desktop", "code", "6IMPOSE")

    def __init__(self, cb_camera: Callable[[CamFrame], None] = None):
        self.settings = App().settings['Object Detection']
        self.t_last_cb_camera = 0.0

        self.eye_in_hand = D415(name="realsense_D415")
        
        self.eye_in_hand.start()
        self.eye_in_hand.attach(self.endeffector, self.eye_in_hand.extrinsic_matrix)
        self.endeffector = TargetModel("fanuc_crx", mesh=load_mesh(os.path.join(self.BASE_PATH, 'j6.ply')))
        
        self.dt = None
        self.cb_camera = cb_camera

        self.thread = Thread(target=self.main)

        self.thread.start()

        self.detector = self.create_detector()

    def create_detector(self):
        from impose_grasp.lib.utils import path_to_demo_file
        darknet_paths = {
            "6IMPOSE": self.SIX_IMPOSE_PATH,
            "yolo_config": path_to_demo_file("yolo_model/yolov4-tiny-lm-cat.cfg"),
            "data_file": path_to_demo_file("yolo_model/single_obj.data"),
            "yolo_weights": path_to_demo_file("yolo_model/yolov4-tiny-lm-cat_best.weights")
        }

        pvn_paths = {
            "6IMPOSE": self.SIX_IMPOSE_PATH,
            "mesh": path_to_demo_file("obj_06.ply"),
            "kpts": path_to_demo_file("farthest.txt"),
            "corner": path_to_demo_file("corners.txt"),
            "config": path_to_demo_file("pvn_model/log/cat/config.json"),
            "weights": path_to_demo_file("pvn_model/model/cat/best_model/model"),
            "demo_data": path_to_demo_file()
        }

        self.detector = PositionDetector(darknet_paths, pvn_paths)


    def manage_fps(self):
        """ Makes the while loop sleep to run at 30fps """
        t_last_cb_camera = self.t_last_cb_camera
        dt = time.perf_counter() - t_last_cb_camera

        if dt < 1/self.settings['FPS']:
            # limit to 30 FPS
            time.sleep(1/self.settings['FPS'] - dt)
            dt = time.perf_counter() - t_last_cb_camera

        self.t_last_cb_camera = time.perf_counter()

        self.dt = dt

    def main(self):
        frame = None

        while True:
            self.manage_fps()

            frame = self.cam.grab_frame()
            if frame is None:
                continue

            self.detector.detect(frame.rgb, frame.depth)

            result_img = self.detector.get_result_img()

            cv2.imshow("detected object", result_img)

            key = cv2.waitKey(1)
            if key == ord('q'):  # Exit loop if 'q' key is pressed
                break

        cv2.destroyAllWindows()

    def close(self):
        print("Quitting...")
        self.thread.join()
        self.cam.close()

ct = CameraThread()
ct.main()
ct.close()