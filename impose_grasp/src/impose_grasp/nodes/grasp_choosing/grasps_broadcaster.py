import rospy
from typing import List
from std_msgs.msg import Bool, Float32

from impose_grasp.lib.tf_broadcaster import TransformBroadcaster
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps


class GraspsBroadcasater(Grasps):
    def __init__(self, obj_frame,  grasps: Grasps) -> None:
        super().__init__()
        self.obj_frame = obj_frame

        self.set_up_br(grasps)

        self.pow_gr_pub = rospy.Publisher('power_gr', Bool, queue_size=10)
        self.widht_pub = rospy.Publisher('gr_width', Float32, queue_size=10)

        self.broadcasters: List[TransformBroadcaster] 
        self.target_gr_br = TransformBroadcaster(obj_frame, "/target_grasp")


    def broadcast_grasps(self):
        for i in range(self.num_grasps):
            self.broadcast_grasp(i)

    def broadcast_target(self, target_ind):
        gpose = self.rel_poses[target_ind]
        self.target_gr_br.broadcast_transform(gpose) 

        self._publish_msgs(target_ind)

    def broadcast_grasp(self, ind):
        gpose = self.rel_poses[ind]
        br = self.broadcasters[ind]
        br.broadcast_transform(gpose)  

        self._publish_msgs(ind)

    
    def set_up_br(self, grasps: Grasps):
        self.set_rel_poses(grasps.rel_poses)
        self.set_widths(grasps.widths)
        self.set_power_gr(grasps.power_gr)

        self.num_grasps = len(self.widths)

        self.broadcasters = []
        for i in range(self.num_grasps):
            grasp_name = "/grasp_n_" + str(i)
            self.broadcasters.append(TransformBroadcaster(self.obj_frame, grasp_name))

    def _publish_msgs(self, ind):
        pow_gr_msg = Bool()
        pow_gr_msg.data = self.power_gr[ind]

        gr_width_msg = Float32()
        gr_width_msg.data = self.widths[ind]

        self.pow_gr_pub.publish(pow_gr_msg)
        self.widht_pub.publish(gr_width_msg)