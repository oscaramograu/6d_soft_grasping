#!/usr/bin/env python  

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from impose_grasp.lib.utils import numpy_to_multiarray
from cv_bridge import CvBridge
import cv2

from impose_grasp.models.cameras.realsense_D415 import D415

def depth_numpy_to_ros(depth_image):
    bridge = CvBridge()
    depth_image_msg = bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")

    # Update the header information as needed
    depth_image_msg.header.frame_id = "camera_frame"

    return depth_image_msg

def set_up_camera_info():
    D = [0.11310159563080885, -0.25086810996017317, 0.00035361841864570614, -0.005393722121755429, 0.0]
    
    K = [1.354081845631475744e+03, 0.000000000000000000e+00, 9.710950536910352184e+02, 0.000000000000000000e+00, 1.354081845631475744e+03, 5.606642250597724342e+02,
0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]
    
    R = [0.37215665516088925, -0.020097006977225797, -0.9279523340829615, 0.05043420784283281, 0.998726400003529, -0.0014030735037909468, 0.9267986915715435, -0.04627837774326621, 0.37269625307309795]
    P = [297.70310942771675, 0.0, -4253.122383117676, 0.0, 0.0, 297.70310942771675, -228.14882278442383, 0.0, 0.0, 0.0, 1.0, 0.0]
    
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
        '/camera/depth/camera_info', CameraInfo, queue_size=10)
    
    rgb_np_pub = rospy.Publisher(
        '/camera/rgb/numpy', Float32MultiArray, queue_size=10)
    d_np_pub = rospy.Publisher(
        '/camera/depth/numpy', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # Publish at 10Hz

    while not rospy.is_shutdown():
        frame = camera.grab_frame()
        bgr_image = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)

        time = rospy.Time.now()

        # Convert the RGB image to ROS message
        rgb_image_msg = bridge.cv2_to_imgmsg(bgr_image, "bgr8")
        rgb_image_msg.header.stamp = time
        # Publish the RGB image
        image_pub_rgb.publish(rgb_image_msg)

        # Convert the depth numpy image to ROS Image message
        d_img_msg = depth_numpy_to_ros(frame.depth)
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

