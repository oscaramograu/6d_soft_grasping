#!/usr/bin/env python  

import rospy
from sensor_msgs.msg import PointCloud2

from impose_grasp.lib.point_cloud import PointCloud
from impose_grasp.nodes.grasp_choosing.grasps_br_node import GraspBrNode


if __name__ == "__main__":
    rospy.init_node("point_cloud_viewer")
    obj = rospy.get_param("/target_object")
    frame = obj + "_frame"

    pub = rospy.Publisher("Realsense/points", PointCloud2, queue_size=1)
    pcd = PointCloud(frame)
    r = rospy.Rate(2) # 10Hz

    # br_node = GraspBrNode(obj)
    # g_pose = br_node.broadcaster.rel_poses[br_node.target]

    n = 0
    while not rospy.is_shutdown() and n<10:
        n+=1

        pcd.set_new_pcd_wrt_obj(0.005)

        pcd_wrt_obj = pcd.obstruction_pcl.cpu().clone()
        ros_pcd = pcd.get_pcd_in_ros(pcd_wrt_obj)

        # pcd_wrt_targ = pcd.get_pcd_wrt_target(g_pose)
        # ros_pcd = pcd.get_pcd_in_ros(pcd_wrt_targ, frame_id=frame)

        pub.publish(ros_pcd)

        r.sleep()

    # print("The pointcloud has been published ", n, " times, check if makes sence")