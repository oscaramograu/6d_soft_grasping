#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

def set_up_camera_info():
    D = [0.11310159563080885, -0.25086810996017317, 0.00035361841864570614, -0.005393722121755429, 0.0]
    K = [598.9326314947814, 0.0, 306.7966418687852, 0.0, 599.5015658716676, 231.988396801157, 0.0, 0.0, 1.0]
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


def camera_node():
    rospy.init_node('camera_node', anonymous=True)

    image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    camera_info_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=10)

    camera_info_msg = set_up_camera_info()

    bridge = CvBridge()

    # Open the camera
    cap = cv2.VideoCapture(4)  # 4 left, 2 right, 0 webcam

    rate = rospy.Rate(30)  # desired publishing rate (30 Hz in this example)
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            frame_id = "camera_frame"
            time_stamp = rospy.Time.now()

            # Publish the image
            image_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
            image_msg.header.frame_id = frame_id
            image_msg.header.stamp = time_stamp
            image_pub.publish(image_msg)

            # Publish the camera info
            camera_info_msg.header.stamp = time_stamp

            camera_info_pub.publish(camera_info_msg)

        rate.sleep()

    cap.release()
    rospy.loginfo('Camera node has been shutdown.')

if __name__ == '__main__':
    try:
        camera_node()
    except rospy.ROSInterruptException:
        pass
