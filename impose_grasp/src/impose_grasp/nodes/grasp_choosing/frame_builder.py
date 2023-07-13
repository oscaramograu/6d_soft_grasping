import rospy
import os
import numpy as np

from impose_grasp.lib.utils import multiarray_to_numpy, PATH_TO_IMPOSE_GRASP
from std_msgs.msg import Float32MultiArray
from impose_grasp.models.cameras.base_camera import CamFrame

class FrameBuilder:
    def __init__(self):
        self.frame = CamFrame()
        cal_path = cal_path = os.path.join(
            PATH_TO_IMPOSE_GRASP, "data", "camera", "realsense_135222065752")
        
        if os.path.exists(os.path.join(cal_path, "intrinsic_matrix.txt")):
                    self.frame.intrinsic = np.loadtxt(
                        os.path.join(cal_path, "intrinsic_matrix.txt"))        
                    
    def get_actual_frame(self):
        self.frame.rgb = self.get_numpy_img('/camera/rgb/numpy')

        self.frame.depth = self.get_numpy_img('/camera/depth/numpy')

        return self.frame

    def get_numpy_img(self, multiarr_topic):
        msg = rospy.wait_for_message(
            multiarr_topic, Float32MultiArray, timeout=10)
        return multiarray_to_numpy(msg)