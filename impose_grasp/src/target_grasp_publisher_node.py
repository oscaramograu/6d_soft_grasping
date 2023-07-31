#!/usr/bin/env python  
import rospy
import numpy as np
from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser
from impose_grasp.nodes.object_detection.transform_broadcaster import TransformBroadcaster

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

def apply_offsets(Rt:np.ndarray, offsets_hand_frame: np.ndarray):
    pose = Rt.copy()
    rotation = pose[:3,:3]
    offsets_world_frame = rotation@offsets_hand_frame
    pose[:3, 3]+=offsets_world_frame
    return pose

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

    offsets = np.array([0.004, 0.0002, 0])
    off_grasp_pose = apply_offsets(grasp_pose, offsets)

    tf_br1 = TransformBroadcaster(obj_frame, "target_grasp")
    tf_br2 = TransformBroadcaster(obj_frame, "target_grasp2")
    

    print(grasp_pose, )
    print(off_grasp_pose)

    while not rospy.is_shutdown():
        tf_br1.broadcast_transform(grasp_pose)
        tf_br2.broadcast_transform(off_grasp_pose)