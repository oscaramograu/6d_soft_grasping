#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

def camera_node():
    # Initialize the ROS node
    rospy.init_node('camera_node', anonymous=True)

    # Set up the camera publisher
    image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    camera_info_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=10)

    # Set the camera parameters (intrinsic calibration)
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = 'camera_frame'  # Set an accurate coordinate frame here
    camera_info_msg.height = 480  # Set the image height
    camera_info_msg.width = 640   # Set the image width
    # Set other camera parameters such as camera matrix, distortion coefficients, etc.

    # Initialize the OpenCV bridge
    bridge = CvBridge()

    # Open the camera
    cap = cv2.VideoCapture(4)  # Replace 0 with the camera index if multiple cameras are available

    # Start publishing the images and camera info
    rate = rospy.Rate(30)  # Set the desired publishing rate (30 Hz in this example)
    while not rospy.is_shutdown():
        # Capture the image from the camera
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
            camera_info_msg.header.frame_id = frame_id
            # Set the intrinsic matrix
            camera_info_msg.K = [8.933e+02, 0.0, 6.318e+02,
                            0.0,8.933e+02, 3.581e+02,
                            0.0, 0.0, 1.0]
            # Set the distortion coefficients
            camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]

            camera_info_pub.publish(camera_info_msg)

        rate.sleep()

    # Release the camera and shutdown the node
    cap.release()
    rospy.loginfo('Camera node has been shutdown.')

if __name__ == '__main__':
    try:
        camera_node()
    except rospy.ROSInterruptException:
        pass
