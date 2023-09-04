#!/usr/bin/env python  
import rospy
from impose_grasp.nodes.grasp_choosing.grasps_br_node import GraspBrNode

if __name__ == "__main__":
    rospy.init_node('fake_grasp_br_node')
    rate = rospy.Rate(10)  # Hz

    obj = rospy.get_param("/target_object")
    g_br = GraspBrNode(obj)

    while not rospy.is_shutdown():
        g_br(br_only_target=1)
        rate.sleep() 