#!/usr/bin/env python  

import rospy
import ros_numpy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from impose_grasp.lib.utils import numpy_to_multiarray
from cv_bridge import CvBridge
import numpy as np
import cv2

from impose_grasp.models.cameras.realsense_D415 import D415

def depth_numpy_to_ros_img(depth_image):
    bridge = CvBridge()
    depth_image_msg = bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")

    # Update the header information as needed
    depth_image_msg.header.frame_id = "camera_frame"

    return depth_image_msg

def set_up_camera_info():
    D = [0.07338830833962308, -0.14779608743594305, 0.0018903176017166922, 0.0009634899351246804, 0.0]
    K = [1361.2194795600453, 0.0, 950.8232073178983, 0.0, 1363.8946615215057, 534.6843603624786, 0.0, 0.0, 1.0]
    R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P = [1361.2236734776159, 0.0, 952.2484344586607, 0.0, 0.0, 1377.0898584361362, 536.1885389587065, 0.0, 0.0, 0.0, 1.0, 0.0]
    
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = 'camera_frame'
    camera_info_msg.height = 480
    camera_info_msg.width = 640

    # Set the distortion coefficients
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = D
    # Set the intrinsic matrix
    camera_info_msg.K = K
    # Set the rectification coefficients
    camera_info_msg.R = R
    # Set the projection coefficients
    camera_info_msg.P = P
    
    return camera_info_msg

def main():
    rospy.init_node('img_publisher_node', anonymous=True)
    camera = D415(name="realsense_D415")
    camera.start()

    bridge = CvBridge()
    image_pub_rgb = rospy.Publisher(
        '/camera/rgb/image_raw', Image, queue_size=10)
    image_pub_d = rospy.Publisher(
        '/camera/depth/image_raw', Image, queue_size=10)
    
    camera_info_pub = rospy.Publisher(
        '/camera/rgb/camera_info', CameraInfo, queue_size=10)
    
    rgb_np_pub = rospy.Publisher(
        '/camera/rgb/numpy', Float32MultiArray, queue_size=10)
    d_np_pub = rospy.Publisher(
        '/camera/depth/numpy', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # Publish at 10Hz

    while not rospy.is_shutdown():
        frame = camera.grab_frame()

        time = rospy.Time.now()

        # Convert the RGB image to ROS message
        rgb_image_msg:Image
        rgb_image_msg = ros_numpy.msgify(Image, frame.rgb, "rgb8")
        rgb_image_msg.header.stamp = time
        rgb_image_msg.header.frame_id = "camera_frame"
    
        # Publish the RGB image
        image_pub_rgb.publish(rgb_image_msg)

        # Convert the depth numpy image to ROS Image message
        d_img_msg:Image
        d_img_msg = ros_numpy.msgify(Image, frame.depth, "32FC1")

        # d_img_msg = depth_numpy_to_ros_img(frame.depth)
        d_img_msg.header.stamp = time
        # Publish the depth image
        image_pub_d.publish(d_img_msg)

        # Setup camera info msg
        camera_info_msg = set_up_camera_info()
        camera_info_msg.header.stamp = time
        camera_info_pub.publish(camera_info_msg)

        # Setup the numpy msg
        depth_multiarr = numpy_to_multiarray(frame.depth)
        rgb_multiarr = numpy_to_multiarray(frame.rgb)

        d_np_pub.publish(depth_multiarr)
        rgb_np_pub.publish(rgb_multiarr)

        rate.sleep()
    
    camera.stop()

if __name__ == "__main__":
    main()

