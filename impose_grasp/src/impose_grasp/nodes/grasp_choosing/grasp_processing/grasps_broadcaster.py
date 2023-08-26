import rospy
from typing import List
from std_msgs.msg import Bool, Float32
from pandaqb_movegroup_control.msg import Sinergies

from impose_grasp.lib.tf_broadcaster import TransformBroadcaster
from impose_grasp.nodes.grasp_choosing.grasps_base import GraspsBase, Grasps


class GraspsBroadcasater(GraspsBase):
    broadcasters: List[TransformBroadcaster] 
    
    def __init__(self, grasps: Grasps) -> None:
        super().__init__(grasps)
        self.obj_frame = self.obj_name + "_frame"

        self.pow_gr_pub = rospy.Publisher('power_gr', Bool, queue_size=10)
        self.widht_pub = rospy.Publisher('gr_width', Float32, queue_size=10)
        self.singergies_pub = rospy.Publisher('sinergies', Sinergies, queue_size=10)

        self.target_gr_br = TransformBroadcaster(self.obj_frame, "/target_grasp")
        self._set_up_broadcasters()

    def broadcast_all_grasps(self):
        for i in range(len(self.rel_poses)):
            self.broadcast_grasp(i)

    def broadcast_good_grasps(self):
        for i in range(len(self.rel_poses)):
            if(self.good_gr_flags[i]):
                self.broadcast_grasp(i)

    def broadcast_target(self, target_ind):
        gpose = self.rel_poses[target_ind]
        self.target_gr_br.broadcast_transform(gpose) 

        self._publish_msgs(target_ind)

    def broadcast_grasp(self, ind):
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


        sinegies_msg = Sinergies()
        sinegies_msg.first_sin = self.synergies_values[ind][0]
        sinegies_msg.second_sin = self.synergies_values[ind][1]

        gr_width_msg = Float32()
        gr_width_msg.data = self.widths[ind]

        self.widht_pub.publish(gr_width_msg)
        self.singergies_pub.publish(sinegies_msg)