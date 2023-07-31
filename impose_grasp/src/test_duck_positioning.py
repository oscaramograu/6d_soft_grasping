#!/usr/bin/env python  
import rospy
import numpy as np
from impose_grasp.nodes.object_detection.transform_broadcaster import TransformBroadcaster
from impose_grasp.lib.tf_listener import TfListener
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
    rospy.init_node('cpsduck_top_frame_pub_node')

    obj_frame = "/cpsduck_frame"
    duck_listener = TfListener(obj_frame)
    duck_listener.listen_tf()

    target_pose = duck_listener.get_np_frame()

    target_pose = rotate_90x(target_pose)
    # target_pose = rotate_90x(target_pose)
    # target_pose = rotate_90x(target_pose)
    target_pose = rotate_90z(target_pose)

    target_pose[2,3]+=0.1

    tf_br = TransformBroadcaster("panda_link0", "target_grasp")
    print(target_pose)
    while not rospy.is_shutdown():
        tf_br.broadcast_transform(target_pose)