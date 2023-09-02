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
        self._add_pose(pose)

        if len(self.poses) > self.max_num:
            self.poses = self.poses[1:]

    def _add_pose(self, new_pose):
        if len(self.poses) == 0:
            self.poses.append(new_pose)
        else:
            average_pose = self.get_pose_average()
            x_err = average_pose[0, 3] - new_pose[0, 3]
            y_err = average_pose[1, 3] - new_pose[1, 3]
            z_err = average_pose[2, 3] - new_pose[2, 3]

            if x_err < 0.005 and y_err < 0.005 and z_err < 0.005:
                self.poses.append(new_pose)
            else:
                self.poses.clear()

    def get_pose_average(self):
        return sum(self.poses)/len(self.poses)
    
    def clear_poses(self):
        self.poses.clear()