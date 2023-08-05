import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

class TfListener:
    def __init__(self, target_frame, base_frame="panda_link0"):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._rate = rospy.Rate(30)  # 10 Hz
        self.duration = rospy.Duration(2.0)
        self.target_frame = target_frame
        self.base_frame = base_frame

        self.target_tf = None
        self.target_np: np.ndarray
        self.target_pose: Pose

    def listen_tf(self):
        self.target_tf = None
        while not rospy.is_shutdown() and self.target_tf is None:
            try:
                self.target_tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.target_frame, rospy.Time(0), self.duration
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("Error occurred while listening to transform: %s", str(e))
            
            self._rate.sleep()

    def get_np_frame(self):
        if self.target_tf is None:
            return None
        else:
            t = self.target_tf.transform.translation
            quat = self.target_tf.transform.rotation

            R_np = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
            t_np = np.array([t.x, t.y, t.z])

            arr = np.eye(4)
            arr[:3, :3] = R_np
            arr[:3, 3] = t_np

            return arr