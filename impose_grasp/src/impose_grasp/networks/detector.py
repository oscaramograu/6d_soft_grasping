from impose_grasp.networks.darknet_detector import DarknetDetector
from impose_grasp.networks.pvn_detector import PvnDetector
import os
import cv2

class PositionDetector():
    def __init__(self, darknet_paths, pvn_paths) -> None:
        self.darknet = DarknetDetector(darknet_paths)
        self.pvn = PvnDetector(pvn_paths)

        self.result_img = None

    def detect(self,rgb, dpt):
        self.darknet.inference(rgb)
        bbox, resnet_input_size = self.darknet.get_resnet_inputs()

        self.pvn.load_inputs(bbox, resnet_input_size)
        self.pvn.inference(rgb, dpt)

        self.result_img = self.pvn.rgb_img_for_visualization()

    def safe_result(self):
        proj_pose_path = os.path.join(self.pvn.paths["demo_data"], "result.jpg")

        cv2.imwrite(proj_pose_path, cv2.cvtColor(self.result_img, cv2.COLOR_RGB2BGR))
        print("The result is saved to: " + proj_pose_path)

    def get_result_img(self):
        return self.result_img