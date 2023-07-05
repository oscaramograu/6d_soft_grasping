import rospy
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R

from impose_grasp.networks.detector import Detector
from impose_grasp.nodes.transform_broadcaster import TransformBroadcaster
from impose_grasp.models.cameras.realsense_D415 import D415

class ObjectBroadcaster(TransformBroadcaster):
    def __init__(self, target_obj: str):
        obj_frame = target_obj + "_frame"
        node_name = target_obj + "_tf_broadcaster"

        super().__init__("camera_frame", obj_frame)

        rospy.init_node(node_name)
        self.rate = rospy.Rate(10)  # publishing rate in Hz

        self._build_cam_and_det(target_obj)
        self.num = 0
        self.rgb = None

    def broadcast_obj_tf(self):
        """
            - Grabs a frame using cam attribute.
            - Sets that frame in the det attribute.
            - A bounding box is computed using darknet.
            - The frame is cropped within the bbox region. 
            And PVN estimates the 6D pose affine matrix 
            (from the camera to the targeted object)
            - The tf frame is broadcasted between the camera_frame,
            and a new 'target'_frame.
        """
        frame = self.cam.grab_frame()
        self.det.set_frame(frame)
        self.det.compute_bbox()
        self.det.compute_affine()

        # self.rgb = frame.rgb

        affine = self.det.get_affine()
        self.broadcast_transform(affine)
        self.rate.sleep()

    def test_broadcaster(self):
        affine_matrix = np.array([[1.0, 0.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0, 0.0],
                                  [0.0, 0.0, 1.0, 0.3],
                                  [0.0, 0.0, 0.0, 1.0]])

        self.broadcast_transform(affine_matrix)
        self.rate.sleep()

    def _build_cam_and_det(self, target_obj):
        """
            - Builds and starts cam attribute. A D415 object used 
            to grab frames.
            - Builds det attribute. A detector object which uses
            darknet and pvn to estimate the 6D pose of the target 
            object.
        """
        try:
            self.cam = D415(name="realsense_D415")
            self.cam.start()
        except:
            print("Camera could not be found")  
        try:
            self.det = Detector(target_obj)
        except:
            print("Detector could not be created")  

#     def _get_camworld_affine(self):
#         listener = tf.TransformListener()
#         rate = rospy.Rate(10.0)
#         if not rospy.is_shutdown():
#             try:
#                 listener.waitForTransform('/world', '/camera_frame', rospy.Time(0), rospy.Duration(4.0))
#                 (trans,rot) = listener.lookupTransform('/world', '/camera_frame', rospy.Time(0))
#                 print(rot)
#                 print(trans)

#                 trans = np.array(trans)
#                 rot = quaternion_rotation_matrix(np.array(rot))
#                 Rt = np.eye(4)

#                 Rt[:3, :3] = rot
#                 Rt[:3, 3] = trans

#             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                 print("No transform found")


#         return Rt
    
# def quaternion_rotation_matrix(Q):
#     """
#     Covert a quaternion into a full three-dimensional rotation matrix.
 
#     Input
#     :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
#     Output
#     :return: A 3x3 element matrix representing the full 3D rotation matrix. 
#              This rotation matrix converts a point in the local reference 
#              frame to a point in the global reference frame.
#     """
#     # Extract the values from Q
#     q0 = Q[0]
#     q1 = Q[1]
#     q2 = Q[2]
#     q3 = Q[3]
     
#     # First row of the rotation matrix
#     r00 = 2 * (q0 * q0 + q1 * q1) - 1
#     r01 = 2 * (q1 * q2 - q0 * q3)
#     r02 = 2 * (q1 * q3 + q0 * q2)
     
#     # Second row of the rotation matrix
#     r10 = 2 * (q1 * q2 + q0 * q3)
#     r11 = 2 * (q0 * q0 + q2 * q2) - 1
#     r12 = 2 * (q2 * q3 - q0 * q1)
     
#     # Third row of the rotation matrix
#     r20 = 2 * (q1 * q3 - q0 * q2)
#     r21 = 2 * (q2 * q3 + q0 * q1)
#     r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
#     # 3x3 rotation matrix
#     rot_matrix = np.array([[r00, r01, r02],
#                            [r10, r11, r12],
#                            [r20, r21, r22]])
                            
#     return rot_matrix