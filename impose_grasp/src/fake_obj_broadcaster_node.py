#!/usr/bin/env python  

import rospy
import numpy as np
from impose_grasp.lib.tf_broadcaster import TransformBroadcaster
from datetime import datetime

def build_affine_mat()->np.ndarray:
    array = np.array([
        [1, 0, 0, .55],
        [0, 1, 0, 0],
        [0, 0, 1, 0.175],
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

def run(tf_br: TransformBroadcaster, r:rospy.Rate):
    fake_obj_pose = build_affine_mat()
    fake_obj_pose = rotate_around_x(fake_obj_pose)
    fake_obj_pose = rotate_around_y(fake_obj_pose)
    fake_obj_pose = rotate_around_y(fake_obj_pose)

    print("Fake duck pose:\n", fake_obj_pose)

    while not rospy.is_shutdown():
        tf_br.broadcast_transform(fake_obj_pose)
        r.sleep()
        
if __name__ == "__main__":
    rospy.init_node("fake_obj_broadcaster")

    obj = rospy.get_param("/target_object")

    obj_frame_name = "/" + obj + "_frame"
    tf_br = TransformBroadcaster("/panda_link0", obj_frame_name)
    r = rospy.Rate(10)
    run(tf_br, r)