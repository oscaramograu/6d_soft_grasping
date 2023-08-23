from typing import List
import numpy as np 

class PoseFilter:
    def __init__(self, max_num = 10) -> None:
        self.poses:List[np.ndarray] = []
        self.max_num = max_num
        self.f_pose = np.ndarray

    def filter_pose(self, pose: np.ndarray):
        """
        Takes a new pose, computes the average of it toguether with the last safed 
        poses to compute the new filtered one.
        Removing the first pose if the maximum number of poses is reached.
        """
        self.poses.append(pose)

        if len(self.poses) > self.max_num:
            self.poses = self.poses[1:]

        self.f_pose = sum(self.poses)/len(self.poses)

    def get_filtered_pose(self):
        """
        It returns the last filtered pose.
        """
        return self.f_pose
    
    def clear_poses(self):
        self.poses.clear()