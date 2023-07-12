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
        rgb_msg = rospy.wait_for_message(
            '/camera/rgb/numpy', Float32MultiArray, timeout=10)
        rgb = multiarray_to_numpy(rgb_msg)
        self.frame.rgb = rgb

        depth_msg = rospy.wait_for_message(
            '/camera/depth/numpy', Float32MultiArray, timeout=10)
        depth = multiarray_to_numpy(depth_msg)
        self.frame.depth = depth

        return self.frame
        
        # self.rgb_np_sub = rospy.Subscriber(
        #     '/camera/rgb/numpy', Float32MultiArray, self.rgb_callback)
        # self.d_np_sub = rospy.Subscriber(
        #     '/camera/depth/numpy', Float32MultiArray, self.depth_callback)


    # def rgb_callback(self, msg):
    #     rgb = multiarray_to_numpy(msg)
    #     self.frame.rgb = rgb

    # def depth_callback(self, msg):
    #     depth = multiarray_to_numpy(msg)
    #     self.frame.depth = depth