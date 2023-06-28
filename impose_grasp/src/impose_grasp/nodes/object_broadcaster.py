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
        self.rate = rospy.Rate(10)  # Adjust the publishing rate as per your requirement

        try:
            self.cam = D415(name="realsense_D415")
            self.cam.start()
            self.det = Detector(target_obj)

        except:
            print("Camera could not be found")        
        
    def broadcast_obj_tf(self):
        frame = self.cam.grab_frame()

        self.det.set_frame(frame)
        self.det.bbox()
        self.det.affine_matrix()

        affine_matrix = self.det.get_affine()

        self.broadcast_transform(affine_matrix)
        self.rate.sleep()

    def test_broadcaster(self):
        affine_matrix = np.array([[1.0, 0.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0, 0.0],
                                  [0.0, 0.0, 1.0, 0.3],
                                  [0.0, 0.0, 0.0, 1.0]])

        self.broadcast_transform(affine_matrix)
        self.rate.sleep()