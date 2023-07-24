#!/usr/bin/env python  
import rospy
import numpy as np
import random
from impose_grasp.nodes.grasp_choosing.grasp_filterer import GraspFilterer
from impose_grasp.lib.grasp_marker_plotter import MarkerPlotter
from impose_grasp.lib.tf_listener import TfListener
from geometry_msgs.msg import Pose
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

if __name__ == "__main__":
    rospy.init_node('grasp_plotter_node')

    obj = "cpsduck"

    obj_tf_lsitener = TfListener("/" + obj + "_frame")
    obj_tf_lsitener.listen_tf()
    obj_pose = obj_tf_lsitener.get_np_frame()

    cam_tf_lsitener = TfListener("/camera_frame")
    cam_tf_lsitener.listen_tf()
    cam_pose = obj_tf_lsitener.get_np_frame()

    grasp_filterer = GraspFilterer(obj, obj_pose, cam_pose)
    good_grasps = grasp_filterer.get_good_grasps()
    bad_grasps = grasp_filterer.get_bad_grasps()

    marker_plotter = MarkerPlotter()
    for i in range(len(good_grasps)):
        gpose = good_grasps[i][0]
        print(gpose[:3,3])
        marker_plotter.add_marker(gpose, True)
    
    for i in range(len(bad_grasps)):
        gpose = bad_grasps[i][0]
        marker_plotter.add_marker(gpose, False)

    rand_ind = random.randrange(0, len(good_grasps))
    target_grasp = good_grasps[rand_ind][0]

    # The lines below will plot a random valid grasp
    grasp_pose = rotate_90x(target_grasp)
    grasp_pose = rotate_90x(grasp_pose)
    grasp_pose = rotate_90x(grasp_pose)
    grasp_pose = rotate_90z(grasp_pose)
    
    print("Target grasp: \n", target_grasp)
    tf_br = TransformBroadcaster("/panda_link0", "/target_grasp")
    while not rospy.is_shutdown():
        tf_br.broadcast_transform(target_grasp)

    # Delete the lines above to visualize array
    marker_plotter.visualize_array()

