#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pandaqb_movegroup_control.srv._RequestGrasp import RequestGrasp, RequestGraspResponse
import cv2


class CameraComunicator():
    def __init__(self):
        # Subscribe to the grasp service
        self.grasp_server = rospy.Service('/RequestGrasp',RequestGrasp,self.detect_grasp_callback)
        rospy.loginfo("The Service /RequestGrasp is ready!")

        # Subscribe to the rgb image topic of the ZED mini camera
        self.rgb_subscriber = rospy.Subscriber('rgb/image_rect_color', Image, self.rgb_callback)
        rospy.loginfo("The Subscriber to ZED rgb image is ready!")

        self.rgb = None
        self.bridge = CvBridge()

        self.set_params()

    def detect_grasp_callback(self, req):
        rospy.loginfo("Grasp requested from Controller Node to NN Node")
        response = RequestGraspResponse()

        # Set the response of the grasp request
        self.set_params()
        response.point = self.GraspPoint
        response.theta = self.theta
        response.w = self.w

         # Display RGB image
        cv2.imshow('RGB Image', self.rgb_image)
        cv2.waitKey(1)
        
        return response
    
    def rgb_callback(self, rgb_msg):
        try:
            # Convert ROS image message to OpenCV image
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)
            return


    def set_params(self):
        self.GraspPoint = Point()
        self.GraspPoint.x = rospy.get_param('fake_params/grasp_position')[0]
        self.GraspPoint.y = rospy.get_param('fake_params/grasp_position')[1]
        self.GraspPoint.z = rospy.get_param('fake_params/grasp_position')[2]

        self.theta = rospy.get_param('fake_params/theta')
        self.w = rospy.get_param('fake_params/w')
        rospy.loginfo("Parameters set from the parameter server.")