#!/usr/bin/env python  

import rospy
from sensor_msgs.msg import PointCloud2

from impose_grasp.nodes.visualisation.point_cloud import PointCloud
from impose_grasp.nodes.grasp_choosing.grasp_filterer import GraspFilterer
from impose_grasp.nodes.grasp_choosing.grasps_broadcaster import GraspsBroadcasater

if __name__ == "__main__":
    rospy.init_node("point_cloud_viewer")
    pub = rospy.Publisher("Realsense/points", PointCloud2, queue_size=1)
    pcd = PointCloud("cpsduck_frame")
    r = rospy.Rate(2) # 10Hz

    g_chooser = GraspFilterer()
    g_pose = g_chooser.rel_poses[0]
    g_w = g_chooser.widths[0]
    g_br = GraspsBroadcasater(g_chooser)

    while not rospy.is_shutdown():
        g_br.broadcast_target(0)

        pcd.set_new_pcd_wrt_obj(0)
        pcd_wrt_obj = pcd.obstruction_pcl.cpu().clone()
        pcd_wrt_targ = pcd.get_pcd_wrt_target(g_pose, g_w)

        ros_pcd = pcd.get_pcd_in_ros(pcd_wrt_obj)
        # ros_pcd = pcd.get_pcd_in_ros(pcd_wrt_targ, frame_id="cpsduck_frame")

        pub.publish(ros_pcd)

        r.sleep()