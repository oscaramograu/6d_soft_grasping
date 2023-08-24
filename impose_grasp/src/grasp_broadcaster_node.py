#!/usr/bin/env python  
import rospy
from impose_grasp.nodes.grasp_choosing.grasps_br_node import GraspBrNode

# def filter_grasps(obj_pose, grasps=None):
#     g_filt = GraspFilterer(obj_pose, grasps)
#     g_filt.filter()

#     good_grasps, good_ids = g_filt.get_good_grasps()
#     if len(good_grasps.power_gr) == 0:
#         print("The length of the good grasps was 0.")
#         good_grasps = g_filt

#     return good_grasps, good_ids


if __name__ == "__main__":
    rospy.init_node('fake_grasp_br_node')
    rate = rospy.Rate(20)  # Hz

    obj = rospy.get_param("/target_object")
    g_br = GraspBrNode(obj)

    # print("The good grasps ids are: ", good_ids)
    # print("The  selected grasp is: ", good_ids[ind])
    # print("The power grasp flag is: ", g_br.power_gr[ind])
    # print("The power grasp value is: ", g_br.widths[ind])
    # print("The pose of the target grasp is: ", g_br.rel_poses[ind])

    while not rospy.is_shutdown():
        g_br(br_only_target=False)
        rate.sleep() 