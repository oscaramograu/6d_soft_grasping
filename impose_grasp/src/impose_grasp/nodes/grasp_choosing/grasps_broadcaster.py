import numpy as np
import rospy

import random
from typing import List

from impose_grasp.lib.tf_broadcaster import TransformBroadcaster
from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps


class GraspsBroadcasater(Grasps):
    def __init__(self, grasps: Grasps = None) -> None:
        super().__init__()
        if grasps is not None:
            self.set_up_br(grasps)
        self.broadcasters: List[TransformBroadcaster] 

        self._rate = rospy.Rate(0.5)  # Hz

    def broadcast_grasps(self):
        while not rospy.is_shutdown():
            for i in range(self.num_grasps):
                pose = self.rel_poses[i]
                broadcaster = self.broadcasters[i]
                broadcaster.broadcast_transform(pose)
            
            self._rate.sleep()  

    def broadcast_random_grasp(self):
        rand_ind = random.randrange(0, self.num_grasps)
        gpose = self.rel_poses[rand_ind]

        tf_br = TransformBroadcaster("/cpsduck_frame", "/target_grasp")

        while not rospy.is_shutdown():
            tf_br.broadcast_transform(gpose)

    def broadcast_target_grasp(self):
        gr_chooser = GraspChooser("cpsduck")
        grasp_pose = gr_chooser.compute_best_grasp_pose()

        tf_br = TransformBroadcaster("/cpsduck_frame", "/grasp")

        while not rospy.is_shutdown():
            tf_br.broadcast_transform(grasp_pose)     
    
    def set_up_br(self, grasps: Grasps):
        self.set_rel_poses(grasps.rel_poses)
        self.set_widths(grasps.widths)
        self.num_grasps = len(self.widths)

        self.broadcasters = []
        for i in range(self.num_grasps):
            grasp_name = "/grasp_n_" + str(i)
            self.broadcasters.append(TransformBroadcaster("/cpsduck_frame", grasp_name))
