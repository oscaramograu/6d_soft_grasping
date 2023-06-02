import cv2
import time

from impose_grasp.app.app import App
from impose_grasp.models.cameras.realsense_D415 import D415
from impose_grasp.networks.detector import PositionDetector

class CameraThread:
    SIX_IMPOSE_PATH = "/home/oscar/Desktop/code/6IMPOSE" 

    def __init__(self):
        self._settings = App().settings['Object Detection']
        self._t_last_cb_camera = 0.0

        self._cam = D415(name="realsense_D415")
        
        self._cam.start()
        
        self._dt = None

        self.detector = self._create_detector()

    def _create_detector(self) -> PositionDetector:
        """ 
            Loads the paths of the files that are necessary to create the darknet detector 
            and for the pvn pose estimator.
            - Darknet: 
                - yolo config
                - obj data
                - yolo pre-trained weights
            - PVN: 
                - object mesh
                - keypoints
                - corners
                - pvn config
                - pvn pre-trained weights    
        """

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

        return PositionDetector(darknet_paths, pvn_paths)


    def _manage_fps(self, fps: int) -> None:
        """ Makes the loop sleep to run at the defined FPS """
        t_last_cb_camera = self._t_last_cb_camera
        dt = time.perf_counter() - t_last_cb_camera

        if dt < 1/fps:
            # limit to 30 FPS
            time.sleep(1/fps - dt)
            dt = time.perf_counter() - t_last_cb_camera

        self._t_last_cb_camera = time.perf_counter()

        self._dt = dt

    def main(self) -> None:
        """ 
            Executes a while loop running at the fps determined in the settings file.
            It positions the object in the 3D space using darknet and pvn.
        """
        frame = None
        while True:
            self._manage_fps(self._settings['FPS'])
            print(self._settings['FPS'])

            frame = self._cam.grab_frame()
            if frame is None:
                continue

            self.detector.detect(frame.rgb, frame.depth)

            result_img = self.detector.get_result_img()

            cv2.imshow("detected object", result_img)

            key = cv2.waitKey(1)
            if key == ord('q'):  # Exit loop if 'q' key is pressed
                break

        cv2.destroyAllWindows()

        self._close()

    def _close(self):
        """ Turns the camera off """
        print("Quitting...")
        self._cam.close()