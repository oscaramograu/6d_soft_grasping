import rospy
from typing import List
from pandaqb_movegroup_control.msg import Grasp

from impose_grasp.lib.tf_broadcaster import TransformBroadcaster
from impose_grasp.nodes.grasp_choosing.grasps_base import GraspsBase, Grasps


class GraspsBroadcasater(GraspsBase):
    broadcasters: List[TransformBroadcaster] 
    
    def __init__(self, grasps: Grasps) -> None:
        super().__init__(grasps)
        self.obj_frame = self.obj_name + "_frame"

        self.grasp_params_pub = rospy.Publisher('grasp_params', Grasp, queue_size=10)

        self.target_gr_br = TransformBroadcaster(self.obj_frame, "/target_grasp")
        self._set_up_broadcasters()

    def broadcast_good_grasps(self):
        for i in range(len(self.rel_poses)):
            # if(self.good_gr_flags[i]):
            self._broadcast_grasp(i)

    def broadcast_target(self, target_ind):
        gpose = self.rel_poses[target_ind]
        self.target_gr_br.broadcast_transform(gpose) 

        self._publish_msgs(target_ind)

    def _broadcast_grasp(self, ind):
        gpose = self.rel_poses[ind]
        br:TransformBroadcaster = self.broadcasters[ind]
        br.broadcast_transform(gpose)  

        self._publish_msgs(ind)
    
    def _set_up_broadcasters(self):
        self.broadcasters = []
        for i in range(len(self.rel_poses)):
            grasp_name = "/grasp_n_" + str(i)
            self.broadcasters.append(TransformBroadcaster(self.obj_frame, grasp_name))

    def _publish_msgs(self, ind):
        grasp_msg = Grasp()
        grasp_msg.sinergies = self.synergies_values[ind]
        grasp_msg.width = self.widths[ind]

        self.grasp_params_pub.publish(grasp_msg)