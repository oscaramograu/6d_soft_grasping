import numpy as np
import rospy

import random
from typing import List

from impose_grasp.nodes.grasp_choosing.grasps_mapper import GraspMapper
from impose_grasp.lib.tf_broadcaster import TransformBroadcaster
from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser


class GraspsBroadcasater:
    def __init__(self, grasps: List) -> None:
        self.grasp_poses_np: List[np.ndarray] = []
        self.broadcasters: List[TransformBroadcaster] = []
        self.num_grasps = len(grasps)
        self._rate = rospy.Rate(1)  # Hz

        self.mapper = GraspMapper(theta=-30, offsets=np.array([0, 0.02, 0]))


        for i in range(self.num_grasps):
            # print(len(grasps))
            # print(self.num_grasps)

            self.grasp_poses_np.append(grasps[i][0])
            grasp_name = "/grasp_n_" + str(i)
            self.broadcasters.append(TransformBroadcaster("/cpsduck_frame", grasp_name))

    def broadcast_grasps(self):
        while not rospy.is_shutdown():
            for i in range(self.num_grasps):
                pose = self.grasp_poses_np[i]
                broadcaster = self.broadcasters[i]
                broadcaster.broadcast_transform(pose)
            
            self._rate.sleep()

    def broadcast_random_grasp(self):
        rand_ind = random.randrange(0, self.num_grasps)
        gpose = self.grasp_poses_np[rand_ind]

        mapped_g = self.mapper.map_grasp([gpose, 0])

        map_tf_br = TransformBroadcaster("/cpsduck_frame", "/mapped_gasp")
        tf_br = TransformBroadcaster("/cpsduck_frame", "/grasp")

        while not rospy.is_shutdown():
            tf_br.broadcast_transform(gpose)
            map_tf_br.broadcast_transform(mapped_g[0])

    def broadcast_target_grasp(self):
        gr_chooser = GraspChooser("cpsduck")
        grasp_pose = gr_chooser.compute_best_grasp_pose()
        mapped_g = self.mapper.map_grasp([grasp_pose, 0])

        tf_br = TransformBroadcaster("/cpsduck_frame", "/grasp")

        while not rospy.is_shutdown():
            tf_br.broadcast_transform(mapped_g[0])     
