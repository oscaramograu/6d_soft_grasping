#!/usr/bin/env python  

import rospy

from impose_grasp.nodes.visualisation.point_cloud import PointCloud, PointCloud2

if __name__ == "__main__":
    rospy.init_node("point_cloud_viewer")
    pub = rospy.Publisher("Realsense/points", PointCloud2, queue_size=1)
    pcd = PointCloud("cpsduck_frame")
    r = rospy.Rate(2) # 10Hz

    while not rospy.is_shutdown():
        pcd.build_pcd_wrt_obj(voxel_size=0.05)
        ros_pcd = pcd.get_pcd_in_ros()
        pub.publish(ros_pcd)

        r.sleep()