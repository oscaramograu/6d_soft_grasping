import rospy
import numpy as np

from impose_grasp.nodes.grasp_choosing.frame_builder import FrameBuilder
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

        self.num = 0
        self.rgb = None

        self.det = Detector(target_obj)
        self.fb = FrameBuilder()

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
        frame = self.fb.get_actual_frame()
        self.det.set_frame(frame)
        self.det.compute_bbox()
        self.det.compute_affine()

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