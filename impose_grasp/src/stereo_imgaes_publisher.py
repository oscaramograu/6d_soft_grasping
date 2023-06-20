#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

left_camera_info = CameraInfo()
right_camera_info = CameraInfo()

def set_camera_info_callback(req):
    global left_camera_info, right_camera_info

    # Retrieve the camera info from the service request
    left_camera_info = req.camera_info
    right_camera_info = req.camera_info

    # Return success response
    return SetCameraInfoResponse(success=True, status_message="Camera info set successfully")

def publish_stereo_images():
    # Initialize ROS node
    rospy.init_node('stereo_camera_publisher')

    # Create publishers for left and right images
    left_image_pub = rospy.Publisher('/left/image_raw', Image, queue_size=10)
    right_image_pub = rospy.Publisher('/right/image_raw', Image, queue_size=10)

    # Create publishers for left and right camera info
    left_camera_info_pub = rospy.Publisher('/left/camera_info', CameraInfo, queue_size=10)
    right_camera_info_pub = rospy.Publisher('/right/camera_info', CameraInfo, queue_size=10)

    # Create a service to set camera info
    set_camera_info_left_service = rospy.Service('left/set_camera_info', SetCameraInfo, set_camera_info_callback)
    set_camera_info__right_service = rospy.Service('right/set_camera_info', SetCameraInfo, set_camera_info_callback)

    # Set camera parameters
    image_width = 640
    image_height = 480

    # Set image capture device IDs
    left_camera_id = 4
    right_camera_id = 2

    # Create OpenCV capture objects
    left_capture = cv2.VideoCapture(left_camera_id)
    right_capture = cv2.VideoCapture(right_camera_id)

    # Set capture properties
    left_capture.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
    left_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
    right_capture.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
    right_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)

    # Create a CvBridge object
    bridge = CvBridge()

    # Main loop
    rate = rospy.Rate(30)  # Set the desired publishing rate (in Hz)
    while not rospy.is_shutdown():
        # Capture left and right images
        _, left_frame = left_capture.read()
        _, right_frame = right_capture.read()

        # Convert OpenCV images to ROS Image messages
        left_image_msg = bridge.cv2_to_imgmsg(left_frame, 'bgr8')
        right_image_msg = bridge.cv2_to_imgmsg(right_frame, 'bgr8')

        # Set identical timestamps for left and right images
        timestamp = rospy.Time.now()
        left_image_msg.header.stamp = timestamp
        right_image_msg.header.stamp = timestamp

        # Publish the left and right images
        left_image_pub.publish(left_image_msg)
        right_image_pub.publish(right_image_msg)

        # Publish the left and right camera info
        left_camera_info.header.stamp = timestamp
        right_camera_info.header.stamp = timestamp

        left_camera_info_pub.publish(left_camera_info)
        right_camera_info_pub.publish(right_camera_info)

        rate.sleep()

    # Release the capture objects
    left_capture.release()
    right_capture.release()

if __name__ == '__main__':
    try:
        publish_stereo_images()
    except rospy.ROSInterruptException:
        pass
