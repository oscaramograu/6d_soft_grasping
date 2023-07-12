import rospy
import tf
from geometry_msgs.msg import Pose
from typing import List

class TfPoseListener:
    def __init__(self, target_object_name):
        self._target = target_object_name

        rospy.init_node('tf_pose_listener_node')
        
        self._tf_listener = tf.TransformListener()
        
        self._cam_pose: Pose = None
        self._obj_pose: Pose = None

        self._rate = rospy.Rate(10)  # 10 Hz
        
    def _get_pose_from_tf(self):
        """
        - Read the pose from the transform tree for both camera, and oject
        frames. 
        - Converts the tf to a pose msg.
        - Saves them into attribute values. _cam_pose and _obj_pose.
        """
        while not rospy.is_shutdown():
            try:
                time = rospy.Time(0)
                camera_tf = self._tf_listener.lookupTransform(
                    '/camera_frame', '/panda_link0', time)
                object_tf = self._tf_listener.lookupTransform(
                    '/' + self._target + '_frame', '/panda_link0', time)
            except (tf.LookupException, tf.ConnectivityException, 
                    tf.ExtrapolationException):
                continue

            self._cam_pose = self._tf_to_pose(camera_tf, "camera_frame")
            self._obj_pose = self._tf_to_pose(object_tf, "object_frame")
            
            self._rate.sleep()

            if (self._cam_pose is not None) and (self._obj_pose is not None):
                rospy.signal_shutdown()

    def _tf_to_pose(self, tf: List[List]):
        t = tf[0]
        rot = tf[1]

        pose = Pose()

        pose.position.x = t[0]
        pose.position.y = t[1]
        pose.position.z = t[2]

        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

        return pose