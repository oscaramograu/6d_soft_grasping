import numpy as np
import rospy
from impose_grasp.nodes.object_detection.transform_broadcaster import TransformBroadcaster
from typing import List

class GraspsBroadcasater:
    def __init__(self, grasps: List) -> None:
        self.grasp_poses_np: List[np.ndarray] = []
        self.broadcasters: List[TransformBroadcaster] = []
        self.num_grasps = len(grasps)
        self._rate = rospy.Rate(1)  # Hz

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