#! /usr/bin/env python

import rospy
from Help.GraspServer import GraspServer


if __name__ == '__main__':
    rospy.init_node('NN_node') 
    rospy.loginfo("The NN_node is ready!")

    server = GraspServer()

    rospy.spin()