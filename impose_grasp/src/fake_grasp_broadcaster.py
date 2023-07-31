#!/usr/bin/env python  
import rospy
import random

from impose_grasp.nodes.grasp_choosing.grasps_mapper import GraspMapper
from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser
from impose_grasp.nodes.grasp_choosing.grasp_filterer import GraspFilterer
from impose_grasp.nodes.grasp_choosing.grasps_broadcaster import GraspsBroadcasater

obj = "cpsduck"

def map_grasps():
    g_map = GraspMapper()
    g_map.load_from_file(obj)
    g_map.map_grasps()
    return g_map

def filter_grasps(grasps):
    g_filt = GraspFilterer(grasps, obj)
    g_filt.filter()
    good_grasps = g_filt.get_good_grasps()

    return good_grasps

def select_target_ind(good_grasps):
    gr_chooser = GraspChooser("cpsduck")
    return gr_chooser.compute_best_grasp_ind()

if __name__ == "__main__":
    rospy.init_node('fake_grasp_br_node')
    rate = rospy.Rate(10)  # Hz

    gr_mapped = map_grasps()
    good_grasps = filter_grasps(gr_mapped)

    # ind = select_target_ind(good_grasps)
    ind = random.randrange(0, len(good_grasps.rel_poses))

    g_br = GraspsBroadcasater(good_grasps)
    while not rospy.is_shutdown():
        g_br.broadcast_grasp(ind)
        rate.sleep() 