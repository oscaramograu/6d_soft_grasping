#!/usr/bin/env python  

import rospy
from impose_grasp.lib.utils import multiarray_to_numpy
from impose_grasp.models.cameras.base_camera import CamFrame
from std_msgs.msg import Float32MultiArray

def callback(msg: Float32MultiArray):
    np_array = multiarray_to_numpy(msg)


def main():
    rospy.init_node('img_listener_node', anonymous=True)

    rgb_np_sub = rospy.Subscriber(
        '/camera/rgb/numpy', Float32MultiArray, callback)
    d_np_sub = rospy.Subscriber(
        '/camera/depth/numpy', Float32MultiArray, callback)   
    
    rospy.spin()

if __name__ == "__main__":
    main()