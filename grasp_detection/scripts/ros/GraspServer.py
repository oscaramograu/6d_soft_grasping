#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from pandaqb_movegroup_control.srv._RequestGrasp import RequestGrasp, RequestGraspResponse
import cv2


class GraspServer():
    def __init__(self):
        # Subscribe to the grasp service
        self.grasp_server = rospy.Service('/RequestGrasp',RequestGrasp,self.grasp_callback)
        rospy.loginfo("The Service /RequestGrasp is ready!")

    def grasp_callback(self, req):
        rospy.loginfo("Grasp requested from Controller Node to NN Node")
        response = RequestGraspResponse()

        # Set the response of the grasp request
        self.set_params()
        response.point = self.GraspPoint
        response.theta = self.theta
        response.w = self.w

        return response

    def set_params(self):
        self.GraspPoint = Point()
        self.GraspPoint.x = rospy.get_param('fake_params/grasp_position')[0]
        self.GraspPoint.y = rospy.get_param('fake_params/grasp_position')[1]
        self.GraspPoint.z = rospy.get_param('fake_params/grasp_position')[2]

        self.theta = rospy.get_param('fake_params/theta')
        self.w = rospy.get_param('fake_params/w')
        rospy.loginfo("Parameters set from the parameter server.")