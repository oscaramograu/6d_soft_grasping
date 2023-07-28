#!/usr/bin/env python  
import rospy
import numpy as np
from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser
from impose_grasp.lib.tf_broadcaster import TransformBroadcaster

def rotate_90x(pose:np.ndarray):
    rotation = np.array(
        [[1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]])
    return pose@rotation
    
def rotate_90y(pose:np.ndarray):
    rotation = np.array(
        [[0, 0, 1, 0],
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 0, 1]])
    return pose@rotation


def rotate_90z(pose:np.ndarray):
    rotation = np.array(
        [[0, -1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])
    return pose@rotation

if __name__ == "__main__":
    rospy.init_node('tf_pose_listener_node')

    obj = "cpsduck"
    obj_frame = "/" + obj + "_frame"
    gr_chooser = GraspChooser(obj)
    grasp_pose = gr_chooser.compute_best_grasp_pose()    
    grasp_pose = rotate_90x(grasp_pose)
    grasp_pose = rotate_90x(grasp_pose)
    grasp_pose = rotate_90x(grasp_pose)
    grasp_pose = rotate_90z(grasp_pose)
    grasp_pose[2,3]-=0.02

    tf_br = TransformBroadcaster(obj_frame, "target_grasp")
    print(grasp_pose)
    while not rospy.is_shutdown():
        tf_br.broadcast_transform(grasp_pose)