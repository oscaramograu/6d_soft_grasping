#!/usr/bin/env python  

import rospy

from impose_grasp.nodes.visualisation.point_cloud import PointCloud, PointCloud2

if __name__ == "__main__":
    rospy.init_node("point_cloud_viewer")
    pub = rospy.Publisher("Realsense/points", PointCloud2, queue_size=1)
    pcd = PointCloud()
    r = rospy.Rate(2) # 10Hz

    while not rospy.is_shutdown():
        while (pcd.frame.rgb is None) or (pcd.frame.depth is None):
            pass
        pcd.build_new_pcd(0.05)
        pub.publish(pcd.ros_pcl)

        r.sleep()