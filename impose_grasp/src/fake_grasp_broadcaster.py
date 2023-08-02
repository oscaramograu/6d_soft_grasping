#!/usr/bin/env python  
import rospy
import random
import numpy as np

from impose_grasp.nodes.grasp_choosing.grasps_mapper import GraspMapper
from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser
from impose_grasp.nodes.grasp_choosing.grasp_filterer import GraspFilterer
from impose_grasp.nodes.grasp_choosing.grasps_broadcaster import GraspsBroadcasater

obj = "cpsduck"

def map_grasps():
    g_map = GraspMapper(width_proportion_th=0.9, offsets=np.array([-0.03, -0.005, 0]))
    g_map.load_from_file(obj)
    g_map.map_grasps()

    return g_map

def filter_grasps(grasps):
    g_filt = GraspFilterer(grasps, obj)
    g_filt.filter()

    good_grasps = g_filt.get_good_grasps()
    return good_grasps

def select_target_ind(good_grasps):
    gr_chooser = GraspChooser(good_grasps)
    return gr_chooser.compute_best_grasp_ind()

if __name__ == "__main__":
    rospy.init_node('fake_grasp_br_node')
    rate = rospy.Rate(10)  # Hz

    gr_mapped = map_grasps()
    good_grasps = filter_grasps(gr_mapped)

    # ind = select_target_ind(good_grasps)
    ind = random.randrange(0, len(good_grasps.rel_poses))
    # ind = 6 

    g_br = GraspsBroadcasater(good_grasps)
    print("The selected index is: ", ind)
    print("The power grasp flag ist: ", g_br.power_gr[ind])
    while not rospy.is_shutdown():
        g_br.broadcast_target(ind)
        rate.sleep() 