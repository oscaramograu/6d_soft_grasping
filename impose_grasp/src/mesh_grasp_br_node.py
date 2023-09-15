#!/usr/bin/env python  

import rospy
from impose_grasp.nodes.grasp_choosing.grasp_for_collision.grasp_selector import SelectedMeshGraspBr
from pandaqb_movegroup_control.msg import Grasp
 
def main():
    rospy.init_node('target_mesh_gr_publisher', anonymous=True)
    rate = rospy.Rate(10)  # Publish at 10Hz

    obj = rospy.get_param("/target_object")
    gr_br = SelectedMeshGraspBr(obj, 20)
    gr_br.find_targ_pose()
    gr_br.print_line()
    # gr_br.build_all_br()

    grasp_params_pub = rospy.Publisher('grasp_params', Grasp, queue_size=10)
    grasp_msg = Grasp()
    grasp_msg.width = 0.06
    grasp_msg.sinergies = [0.8, 0]

    while not rospy.is_shutdown():
        gr_br.broadcast_target_mesh_grasp()
        # gr_br.broadcast_all()
        grasp_params_pub.publish(grasp_msg)
        rate.sleep()

if __name__ == "__main__":
    main()