#!/usr/bin/env python  
import rospy
from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser


if __name__ == "__main__":
    rospy.init_node('tf_pose_listener_node')

    gr_chooser = GraspChooser("cpsduck")
    grasp_pose = gr_chooser.compute_best_grasp_pose()
    