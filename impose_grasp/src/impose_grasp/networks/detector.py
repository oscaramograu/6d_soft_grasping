import os
import cv2

from impose_grasp.networks.darknet_detector import DarknetDetector
from impose_grasp.networks.pvn_detector import PvnDetector
from impose_grasp.models.cameras.base_camera import CamFrame

class PositionDetector():
    def __init__(self, darknet_paths, pvn_paths) -> None:
        self.darknet = DarknetDetector(darknet_paths)
        self.pvn = PvnDetector(pvn_paths)

        self.result_img = None

    def compute_pose_in_frame(self, frame: CamFrame) -> bool:
        """
        - It first uses daknet to crop the detected object in a bbox of the image.
        
        - If the object is detected:
            - The cropped img is sent to PVN to get the rotation and translation matrices of the object
            - The result image attribute is set to the cropped rgb frame

        - If its not detected:
            - The result image attribute is set to the original rgb frame
        """
        detected = self.darknet.inference(frame.rgb)

        if detected:
            bbox, resnet_input_size = self.darknet.get_resnet_inputs()

            self.pvn.inference(frame.rgb, frame.depth, bbox, resnet_input_size)

            self.result_img = self.pvn.rgb_img_for_visualization()
            
        else:
            self.result_img = frame.rgb
        
    def safe_result(self):
        """
        Safes the result image attribute tot a file under the path: demo_data/result.jpg
        """
        proj_pose_path = os.path.join(self.pvn._paths["demo_data"], "result.jpg")

        cv2.imwrite(proj_pose_path, cv2.cvtColor(self.get_result_img(), cv2.COLOR_RGB2BGR))
        print("The result is saved to: " + proj_pose_path)

    def get_result_img(self):
        """ gets the result image after detection """
        return self.result_img
    
    def get_Rt(self):
        """ Gets the Rt matrix obtained from pvn detection """
        return self.pvn.Rt_pre