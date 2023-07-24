#!/usr/bin/env python  

import rospy
import numpy as np
from impose_grasp.nodes.object_detection.transform_broadcaster import TransformBroadcaster

def build_affine_mat()->np.ndarray:
    array = np.array([
        [1, 0, 0, 0.75],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return array


def rotate_around_x(array: np.ndarray):
    rot = np.array([
        [1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ])
    return array@rot

def rotate_around_y(array: np.ndarray):
    rot = np.array([
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 0, 1]
    ])
    return array@rot

def rotate_aroudn_z(array: np.ndarray):
    rot = np.array([
        [0, -1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return array@rot

def run(tf_br: TransformBroadcaster):
    fake_obj_pose = build_affine_mat()
    fake_obj_pose = rotate_around_x(fake_obj_pose)
    fake_obj_pose = rotate_around_y(fake_obj_pose)
    fake_obj_pose = rotate_around_y(fake_obj_pose)

    print("Fake duck pose:\n", fake_obj_pose)

    while not rospy.is_shutdown():
        tf_br.broadcast_transform(fake_obj_pose)

if __name__ == "__main__":
    rospy.init_node("fake_obj_broadcaster")

    obj_frame_name = "/cpsduck_frame"
    tf_br = TransformBroadcaster("/panda_link0", obj_frame_name)

    run(tf_br)