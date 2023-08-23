#!/usr/bin/env python  
import rospy
import random
import numpy as np
from datetime import datetime

from impose_grasp.nodes.grasp_choosing.grasps_mapper import GraspMapper
from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser
from impose_grasp.nodes.grasp_choosing.grasp_filterer import GraspFilterer
from impose_grasp.nodes.grasp_choosing.grasps_broadcaster import GraspsBroadcasater

def map_grasps(obj):
    g_map = GraspMapper(width_proportion_th=0.9, offsets=np.array([-0.005, 0.02, -0.02]))
    g_map.load_from_file(obj)
    g_map.map_grasps()

    return g_map

def filter_grasps(obj, grasps=None, robot_config="qb_hand"):
    g_filt = GraspFilterer(obj, grasps, robot_config)
    g_filt.filter()

    good_grasps = g_filt.get_good_grasps()
    print(len(good_grasps.power_gr))

    if len(good_grasps.power_gr) == 0:
        good_grasps = g_filt
    print(len(good_grasps.power_gr))

    return good_grasps

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

    gr_mapped = map_if_neces(robot_config)

    good_grasps = filter_grasps(obj, gr_mapped, robot_config)
    
    # ind = select_target_ind(obj, good_grasps, robot_config)
    # ind = random.randrange(0, len(good_grasps.rel_poses))
    ind = 0

    g_br = GraspsBroadcasater(frame, good_grasps)
    print("The selected index is: ", ind)
    print("The power grasp flag is: ", g_br.power_gr[ind])
    print("The pose of the target grasp is: ", g_br.abs_poses)
    while not rospy.is_shutdown():
        old = datetime.now()
        g_br.broadcast_target(ind)
        # g_br.broadcast_grasps()
        new = datetime.now()
        # print(new-old)
        rate.sleep() 