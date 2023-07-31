import rospy
import numpy as np
from datetime import datetime

from impose_grasp.lib.tf_listener import TfListener
from impose_grasp.lib.frame_builder import FrameBuilder
from impose_grasp.networks.detector import Detector
from impose_grasp.nodes.object_detection.transform_broadcaster import TransformBroadcaster

class ObjectBroadcaster(TransformBroadcaster):
    def __init__(self, target_obj: str):
        obj_frame = target_obj + "_frame"

        super().__init__("panda_link0", obj_frame)

        self.rate = rospy.Rate(10)  # publishing rate in Hz

        self.num = 0
        self.rgb = None

        self.det = Detector(target_obj)
        self.fb = FrameBuilder()

        self.obj_world: np.ndarray = None

    def broadcast_obj_tf(self, stop: bool):
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
        if not stop:
            frame = self.fb.get_actual_frame()
            self.det.set_frame(frame)
            self.det.compute_bbox()
            self.det.compute_affine()

            obj_cam = self.det.get_affine()
            cam_world = self.get_cam_world_affine()
            if obj_cam is not None:
                self.obj_world = cam_world@obj_cam

        if self.obj_world is not None:
            self.broadcast_transform(self.obj_world)

    def test_broadcaster(self):
        affine_matrix = np.array([[1.0, 0.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0, 0.0],
                                  [0.0, 0.0, 1.0, 0.3],
                                  [0.0, 0.0, 0.0, 1.0]])

        self.broadcast_transform(affine_matrix)
        self.rate.sleep()

    def get_cam_world_affine(self):
        tf_listener = TfListener("camera_frame")
        tf_listener.listen_tf()
        CamWorld = tf_listener.get_np_frame()
        return CamWorld