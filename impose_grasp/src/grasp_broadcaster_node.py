#!/usr/bin/env python  
import rospy
from impose_grasp.lib.tf_listener import TfListener

if __name__ == "__main__":
    rospy.init_node('tf_pose_listener_node')

    listener = TfListener("qbhand2m1_end_effector_link", "panda_link8")
    listener.listen_tf()
    print(listener.target_tf)