import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf

class TfListener:
    def __init__(self, target_frame, base_frame="panda_link0") -> None:
        self.listener = tf.TransformListener()

        self.target_frame = target_frame
        self.base_frame = base_frame

        self.trans = None
        self.rot = None

    def listen(self):
        self.listener.waitForTransform(self.base_frame, self.target_frame, rospy.Time(), rospy.Duration(4.0))
        self.trans, self.rot = self.listener.lookupTransform(self.base_frame, self.target_frame, rospy.Time(0))

    def get_np_frame(self):
        R_np = R.from_quat(self.rot).as_matrix()
        t_np = np.array(self.trans)

        arr = np.eye(4)
        arr[:3, :3] = R_np
        arr[:3, 3] = t_np
        return arr