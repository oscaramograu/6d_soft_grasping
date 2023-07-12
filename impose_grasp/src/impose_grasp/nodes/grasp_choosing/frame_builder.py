import rospy
from impose_grasp.lib.utils import multiarray_to_numpy
from std_msgs.msg import Float32MultiArray
from impose_grasp.models.cameras.base_camera import CamFrame

class FrameBuilder:
    def __init__(self):
        self.frame: CamFrame = None
        
        rgb_msg = rospy.wait_for_message(
            '/camera/rgb/numpy', Float32MultiArray, timeout=10)
        rgb = multiarray_to_numpy(rgb_msg)
        self.frame.rgb = rgb

        depth_msg = rospy.wait_for_message(
            '/camera/depth/numpy', Float32MultiArray, timeout=10)
        depth = multiarray_to_numpy(depth_msg)
        self.frame.depth = depth
        
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