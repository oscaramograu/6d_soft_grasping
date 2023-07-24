import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

class TfListener:
    def __init__(self, target_frame):
        self._tf_listener = tf.TransformListener()
        self._rate = rospy.Rate(10)  # 10 Hz
        self.duration = rospy.Duration(4.0)
        self.target_frame = target_frame

        self.target_tf = None
        self.target_np: np.ndarray
        self.target_pose: Pose

    def listen_tf(self):
        self.target_tf = None
        while (self.target_tf is None):
            try:
                time = rospy.Time(0)
                self._tf_listener.waitForTransform(
                    '/panda_link0', self.target_frame, time, self.duration)
                self.target_tf = self._tf_listener.lookupTransform(
                    '/panda_link0', self.target_frame, time)

            except (tf.LookupException, tf.ConnectivityException, 
                    tf.ExtrapolationException):
                continue

            self._rate.sleep()

    def get_np_frame(self):
        if self.target_tf == None:
            return None
        else:
            t = self.target_tf[0]
            quat = self.target_tf[1]

            R_np = R.from_quat(quat).as_matrix()
            t_np = np.array(t)

            arr = np.eye(4)
            arr[:3, :3] =  R_np
            arr[:3, 3] = t_np

            return arr
