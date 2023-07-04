import rospy
import numpy as np

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

        self.rgb = frame.rgb

        affine_matrix = self.det.get_affine()
        last_column = affine_matrix[:, -1]

        # Change the symbol of the last column (except the last value)
        last_column[:-1] *= -1

        # Assign the modified column back to the array
        affine_matrix[:, -1] = last_column
        if self.num<3:
            self.num+=1
            
            print(affine_matrix)



        self.broadcast_transform(affine_matrix)
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