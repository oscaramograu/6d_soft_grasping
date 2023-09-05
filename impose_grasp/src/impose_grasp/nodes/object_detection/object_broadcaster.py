import rospy
from datetime import datetime

from impose_grasp.lib.tf_listener import TfListener
from impose_grasp.lib.frame_builder import FrameBuilder
from impose_grasp.networks.detector import Detector
from impose_grasp.lib.tf_broadcaster import TransformBroadcaster
from impose_grasp.nodes.object_detection.pose_filter import PoseFilter

class  ObjectBroadcaster(TransformBroadcaster):
    def __init__(self, target_obj: str, max_poses: int):
        obj_frame = target_obj + "_frame"

        super().__init__("panda_link0", obj_frame)
        self.rate = rospy.Rate(10)  # publishing rate in Hz

        self.num = 0
        self.rgb = None

        self.max_poses = max_poses

        self.det = Detector(target_obj)
        self.fb = FrameBuilder()
        self.tf_listener = TfListener("camera_frame")

        self.filter = PoseFilter(self.max_poses)

        self.start = datetime.now()

    def broadcast_tf(self):
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
        if len(self.filter.poses) < self.max_poses:
            obj_world = self._compute_transform()
            if obj_world is not None:
                self.filter.filter_pose(obj_world)
            if len(self.filter.poses) == self.max_poses:
                print("object has been fixed")
                stop = datetime.now()
                delta_t = stop - self.start
                print("It took:", delta_t.seconds + delta_t.microseconds*10**-6)
                return True
            else:
                return self._check_timeout(45)

        else:
            obj_world = self.filter.get_pose_average()
            self.broadcast_transform(obj_world)
            return True
            
    def _check_timeout(self, max_sec):
        now = datetime.now()
        delta_t = now - self.start
        if delta_t.seconds + delta_t.microseconds*10**-6 > max_sec:
            print("No object found in ", max_sec, "s, timed out!")
            return False
        else:
            return True

    def restart(self):
        self.filter.clear_poses()
        self.start = datetime.now()

    def _compute_transform(self):
        frame = self.fb.get_actual_frame()
        f_obj_cam = self._compute_obj_cam_affine(frame)
        cam_world = self._get_cam_world_affine()

        if f_obj_cam is not None:
            obj_world = cam_world@f_obj_cam
            print("object has been found")
            return obj_world
        else:
            return None

    def _get_cam_world_affine(self):
        self.tf_listener.listen()
        CamWorld = self.tf_listener.get_np_frame()
        return CamWorld

    def _compute_obj_cam_affine(self, frame):
        self.det.set_frame(frame)
        self.det.compute_bbox()
        self.det.compute_affine()

        return self.det.get_affine()