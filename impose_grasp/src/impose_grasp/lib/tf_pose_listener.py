import rospy
import tf
from geometry_msgs.msg import Pose
from typing import List

class TfPoseListener:
    def __init__(self, target_object_name):
        self._target = target_object_name
        
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
        object_frame = '/' + self._target + '_frame'
        duration = rospy.Duration(4.0)

        while (self._cam_pose is None) and (self._obj_pose is None):
            try:
                time = rospy.Time(0)
                camera_tf = self._listen_tf('/camera_frame', time, duration)
                object_tf = self._listen_tf(object_frame, time, duration)
                
            except (tf.LookupException, tf.ConnectivityException, 
                    tf.ExtrapolationException):
                continue
            
            self._cam_pose = self._tf_to_pose(camera_tf)
            self._obj_pose = self._tf_to_pose(object_tf)
            
            self._rate.sleep()

    def _listen_tf(self, frame, time, duration):
        self._tf_listener.waitForTransform(
            '/panda_link0', frame, time, duration)
        
        tf = self._tf_listener.lookupTransform(
            '/panda_link0', frame, time)
        
        return tf

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