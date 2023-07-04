import rospy
import tf
from tf.transformations import quaternion_from_matrix, translation_from_matrix
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
import numpy as np

class TransformBroadcaster:
    def __init__(self, parent_frame: str, child_frame: str):
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.br = tf.TransformBroadcaster()

    def broadcast_transform(self, affine_matrix: np.ndarray):
        """
            - It broadcasts the tf from a parent frame to a child frame,
            given the affine matrix between them.
        """
        if affine_matrix is not None:
            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()

            transform_msg.header.frame_id = self.parent_frame
            transform_msg.child_frame_id = self.child_frame

            rotation, translation = self._extratc_Rt(affine_matrix)
            transform_msg.transform.translation = self._translation_to_tf(translation)
            transform_msg.transform.rotation = self._rotation_to_tf(rotation)

            self.br.sendTransformMessage(transform_msg)

    def _extratc_Rt(self, affine_matrix: np.ndarray):
        rotation = quaternion_from_matrix(affine_matrix)
        translation = translation_from_matrix(affine_matrix)

        return rotation, translation
    
    def _rotation_to_tf(self, rotation):
        rotation_msg = Quaternion()

        rotation_msg.x = rotation[0]
        rotation_msg.y = rotation[1]
        rotation_msg.z = rotation[2]
        rotation_msg.w = rotation[3]

        return rotation_msg
    
    def _translation_to_tf(self, translation):
        translation_msg = Vector3()

        translation_msg.x = translation[0]
        translation_msg.y = translation[1]
        translation_msg.z = translation[2]

        return translation_msg