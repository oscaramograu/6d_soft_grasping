import rospy
import tf
from tf.transformations import quaternion_from_matrix, translation_from_matrix
from geometry_msgs.msg import TransformStamped
import numpy as np

class TransformBroadcaster:
    def __init__(self, parent_frame: str, child_frame: str):
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.br = tf.TransformBroadcaster()

    def broadcast_transform(self, affine_matrix: np.ndarray):
        transform_msg = TransformStamped()
        transform_msg.header.stamp = rospy.Time.now()

        transform_msg.header.frame_id = self.parent_frame
        transform_msg.child_frame_id = self.child_frame

        rotation, translation = self._extratc_Rt(affine_matrix)

        transform_msg.transform.translation.x = translation[0]
        transform_msg.transform.translation.y = translation[1]
        transform_msg.transform.translation.z = translation[2]
        
        transform_msg.transform.rotation.x = rotation[0]
        transform_msg.transform.rotation.y = rotation[1]
        transform_msg.transform.rotation.z = rotation[2]
        transform_msg.transform.rotation.w = rotation[3]

        self.br.sendTransformMessage(transform_msg)

    def _extratc_Rt(self, affine_matrix: np.ndarray):

        rotation = quaternion_from_matrix(affine_matrix)
        translation = translation_from_matrix(affine_matrix)

        return rotation, translation