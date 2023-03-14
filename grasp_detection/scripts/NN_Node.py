#! /usr/bin/env python

import rospy
from CameraComunicator import CameraComunicator


if __name__ == '__main__':
    rospy.init_node('NN_node') 
    rospy.loginfo("The NN_node is ready!")

    cc = CameraComunicator()

    rospy.spin()