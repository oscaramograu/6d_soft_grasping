#!/usr/bin/env python  
import rospy

from impose_grasp.nodes.grasp_choosing.grasps_mapper import GraspMapper
from impose_grasp.nodes.grasp_choosing.grasp_filterer import GraspFilterer
from impose_grasp.nodes.grasp_choosing.grasps_broadcaster import GraspsBroadcasater

obj = "cpsduck"

if __name__ == "__main__":
    rospy.init_node('grasp_plotter_node')

    g_map = GraspMapper()
    g_map.load_from_file(obj)
    g_map.map_grasps()

    g_filt = GraspFilterer(g_map, obj)
    g_filt.filter()
    good_grasps = g_filt.get_good_grasps()

    g_br = GraspsBroadcasater(good_grasps)
    g_br.broadcast_grasps()