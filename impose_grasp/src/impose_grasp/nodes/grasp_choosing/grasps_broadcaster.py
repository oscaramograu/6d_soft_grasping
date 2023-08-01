import rospy
from typing import List
from std_msgs.msg import Bool

from impose_grasp.lib.tf_broadcaster import TransformBroadcaster
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps


class GraspsBroadcasater(Grasps):
    def __init__(self, grasps: Grasps) -> None:
        super().__init__()
        self.set_up_br(grasps)

        self.pow_gr_pub = rospy.Publisher('power_gr', Bool, queue_size=10)
        self.broadcasters: List[TransformBroadcaster] 
        self.target_gr_br = TransformBroadcaster("/cpsduck_frame", "/target_grasp")

    def broadcast_grasps(self):
        for i in range(self.num_grasps):
            self.broadcast_grasp(i)

    def broadcast_target(self, target_ind):
        gpose = self.rel_poses[target_ind]
        self.target_gr_br.broadcast_transform(gpose) 

        self._publish_flag(target_ind)

    def broadcast_grasp(self, ind):
        gpose = self.rel_poses[ind]
        br = self.broadcasters[ind]
        br.broadcast_transform(gpose)  

        self._publish_flag(ind)

    
    def set_up_br(self, grasps: Grasps):
        self.set_rel_poses(grasps.rel_poses)
        self.set_widths(grasps.widths)
        self.set_power_gr(grasps.power_gr)

        self.num_grasps = len(self.widths)

        self.broadcasters = []
        for i in range(self.num_grasps):
            grasp_name = "/grasp_n_" + str(i)
            self.broadcasters.append(TransformBroadcaster("/cpsduck_frame", grasp_name))

    def _publish_flag(self, ind):
        pow_gr_msg = Bool()
        pow_gr_msg.data = self.power_gr[ind]
        self.pow_gr_pub.publish(pow_gr_msg)