#!/usr/bin/env python  
import rospy
from impose_grasp.nodes.grasp_choosing.grasp_filterer import GraspFilterer
from impose_grasp.lib.grasp_marker_plotter import MarkerPlotter
from impose_grasp.lib.tf_listener import TfListener
from geometry_msgs.msg import Pose

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

    marker_plotter.visualize_array()
    # pose1 = Pose()
    # pose1.position.x = 1.0
    # marker_plotter.add_marker(pose1, True)

    # pose2 = Pose()
    # pose2.position.x += 1.5
    # marker_plotter.add_marker(pose2, False)