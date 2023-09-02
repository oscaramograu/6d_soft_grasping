import rospy
import os
import numpy as np


from sensor_msgs.msg import Image
from impose_grasp.lib.utils import multiarray_to_numpy, PATH_TO_IMPOSE_GRASP
from std_msgs.msg import Float32MultiArray
from impose_grasp.models.cameras.base_camera import CamFrame

class FrameBuilder:
    def __init__(self):
        self.frame = CamFrame()

        self.rgb_sub = rospy.Subscriber(
             "/camera/rgb/numpy", Float32MultiArray, self.rgb_callback)
        self.d_sub = rospy.Subscriber(
             "/camera/depth/numpy", Float32MultiArray, self.d_callback)
        
        cal_path = cal_path = os.path.join(
            PATH_TO_IMPOSE_GRASP, "data", "camera", "realsense_135222065752")
        
        if os.path.exists(os.path.join(cal_path, "intrinsic_matrix.txt")):
                    self.frame.intrinsic = np.loadtxt(
                        os.path.join(cal_path, "intrinsic_matrix.txt"))        
                    
    def get_actual_frame(self):
        if self.frame.depth is None:
            rospy.wait_for_message("/camera/depth/numpy", Image)
        if self.frame.rgb is  None:
            rospy.wait_for_message("/camera/rgb/numpy", Image)
        return self.frame

    def rgb_callback(self, msg: Float32MultiArray):
        self.frame.rgb = multiarray_to_numpy(msg)

    def d_callback(self, msg: Float32MultiArray):
        self.frame.depth = multiarray_to_numpy(msg)