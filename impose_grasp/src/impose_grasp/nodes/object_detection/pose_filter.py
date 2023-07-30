from typing import List
import numpy as np 

class PoseFilter:
    def __init__(self, max_num = 10) -> None:
        self.poses:List[np.ndarray] = []
        self.max_num = max_num

    def filter_pose(self, pose: np.ndarray):
        """
        Takes a new pose, computes the average of it toguether with the last safed 
        filtered poses to compute the new filtered one.
        It safes it at the end of the last filtered poses, removing the first one 
        if the maximum number of poses is reached.
        """
        self._append_filtered_pose(pose)

        if len(self.poses) == self.max_num:
            self.poses = self.poses[1:]

    def _append_filtered_pose(self, n_pose):
        """
        Takes the new pose, it makes the average of the last filtered poses and 
        the new pose to compute the new filtered one. 
        The new filtered pose is added to the end of the last filtered poses. 
        """

        f_pose = np.eye(4)
        self.poses.append(n_pose)

        f_pose = sum(self.poses)/len(self.poses)

        self.poses[-1] = f_pose
    
    def get_filtered_pose(self):
        """
        It returns the last filtered pose.
        """
        return self.poses[-1]