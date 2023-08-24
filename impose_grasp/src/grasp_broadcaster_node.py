#!/usr/bin/env python  
import rospy
import random
import numpy as np
from datetime import datetime

from impose_grasp.nodes.grasp_choosing.grasps_orienter import GraspOrienter
from impose_grasp.nodes.grasp_choosing.grasps_mapper import GraspMapper
from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser
from impose_grasp.nodes.grasp_choosing.grasp_filterer import GraspFilterer
from impose_grasp.nodes.grasp_choosing.grasps_broadcaster import GraspsBroadcasater

def orient_grasps():
    g_or = GraspOrienter()
    g_or.load_from_file(obj)
    g_or.orient_grasps()
    obj_pose = g_or.get_obj_pose()
    return g_or, obj_pose

def map_grasps(g_or):
    g_map = GraspMapper(width_th=0.05, theta=30, grasps=g_or)
    # g_map = GraspMapper(width_proportion_th=0.01, offsets=np.array([0.01, 0, -0.025]))

    # g_map.load_from_file(obj)
    g_map.map_grasps()

    return g_map

def filter_grasps(obj_pose, grasps=None):
    g_filt = GraspFilterer(obj_pose, grasps)
    g_filt.filter()

    good_grasps, good_ids = g_filt.get_good_grasps()
    if len(good_grasps.power_gr) == 0:
        print("The length of the good grasps was 0.")
        good_grasps = g_filt

    return good_grasps, good_ids

def select_target_ind(obj, good_grasps, robot_config):
    gr_chooser = GraspChooser(obj, good_grasps, robot_config)
    return gr_chooser.compute_best_grasp_ind()

def map_if_neces(robot_config):
    print("The robot config is: ", robot_config)
    gr_mapped = None

    if robot_config=="qb_hand":
        print("Grasps were mapped")
        gr_mapped = map_grasps(obj)
    elif robot_config == "gripper":
        print("Grasps where not mapped")
    return gr_mapped

if __name__ == "__main__":
    rospy.init_node('fake_grasp_br_node')
    robot_config = rospy.get_param("/robot_config")

    obj = rospy.get_param("/target_object")
    frame = obj + "_frame"
    rate = rospy.Rate(20)  # Hz

    gr_or, obj_pose = orient_grasps()
    gr_mapped = map_grasps(gr_or)
    good_grasps, good_ids = filter_grasps(obj_pose, gr_mapped)
    
    ind = select_target_ind(obj, good_grasps, robot_config)
    # ind = random.randrange(0, len(good_grasps.rel_poses))
    # ind = 0

    g_br = GraspsBroadcasater(frame, good_grasps)
    # print("The good grasps ids are: ", good_ids)
    # print("The  selected grasp is: ", good_ids[ind])
    print("The power grasp flag is: ", g_br.power_gr[ind])
    print("The power grasp value is: ", g_br.widths[ind])
    # print("The pose of the target grasp is: ", g_br.rel_poses[ind])

    while not rospy.is_shutdown():
        g_br.broadcast_target(ind)
        # g_br.broadcast_grasps()
        rate.sleep() 