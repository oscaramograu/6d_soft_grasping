#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from pandaqb_movegroup_control.srv._DetectGrasp import DetectGrasp, DetectGraspResponse

def detect_grasp(request):
    rospy.loginfo("Detection requested from Camera Node to NN Node")

    GraspPoint = Point()
    GraspPoint.x = rospy.get_param('fake_params/grasp_position')[0]
    GraspPoint.y = rospy.get_param('fake_params/grasp_position')[1]
    GraspPoint.z = rospy.get_param('fake_params/grasp_position')[2]

    theta = rospy.get_param('fake_params/theta')
    w = rospy.get_param('fake_params/w')

    response = DetectGraspResponse()
    response.point = GraspPoint
    response.theta = theta
    response.w = w
    
    return response

rospy.init_node('NN_Node') 

my_service = rospy.Service('/DetectGrasp', DetectGrasp, detect_grasp) 
rospy.loginfo("The Service /DetectGrasp is ready!")

rospy.spin()