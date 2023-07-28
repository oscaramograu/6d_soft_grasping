#!/usr/bin/env python  
import rospy
from impose_grasp.nodes.grasp_choosing.grasp_filterer import GraspFilterer
from impose_grasp.lib.tf_listener import TfListener
from impose_grasp.nodes.grasp_choosing.grasps_broadcaster import GraspsBroadcasater

if __name__ == "__main__":
    rospy.init_node('grasp_plotter_node')

    obj = "cpsduck"

    obj_tf_lsitener = TfListener("/" + obj + "_frame")
    obj_tf_lsitener.listen_tf()
    obj_pose = obj_tf_lsitener.get_np_frame()

    cam_tf_lsitener = TfListener("/camera_frame")
    cam_tf_lsitener.listen_tf()
    cam_pose = cam_tf_lsitener.get_np_frame()

    grasp_filterer = GraspFilterer(obj)
    grasp_filterer.filter(cam_pose, obj_pose)

    good_grasps = grasp_filterer.get_good_grasps()
    bad_grasps = grasp_filterer.get_bad_grasps()

    all_grasps = bad_grasps
    for g in good_grasps:
        all_grasps.append(g)

    gbr = GraspsBroadcasater(good_grasps)

    gbr.broadcast_grasps()