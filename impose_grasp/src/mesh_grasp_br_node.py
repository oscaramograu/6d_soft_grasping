#!/usr/bin/env python  

import rospy
from impose_grasp.nodes.grasp_choosing.grasp_for_collision.grasp_selector import SelectedMeshGraspBr

def main():
    rospy.init_node('target_mesh_gr_publisher', anonymous=True)
    rate = rospy.Rate(10)  # Publish at 10Hz

    obj = rospy.get_param("/target_object")
    gr_br = SelectedMeshGraspBr(obj, 30)
    gr_br.find_targ_pose()

    while not rospy.is_shutdown():
        gr_br.broadcast_target_mesh_grasp()
        rate.sleep()

if __name__ == "__main__":
    main()