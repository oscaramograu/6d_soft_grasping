from impose_grasp.networks.darknet import DarknetDetector
from impose_grasp.networks.pvn import PvnDetector

from impose_grasp.models.cameras.base_camera import CamFrame

class Detector():
    def __init__(self, target_obj_name) -> None:
        self.darknet_detector = DarknetDetector(target_obj_name)

        n_points = 512
        self.pvn_detector = PvnDetector(target_obj_name, n_points)

        self.frame: CamFrame
        self.bbox = None
        self.affine = None
        self.detected = None

    def set_frame(self, frame: CamFrame):
        """
            - Sets the atribute frame given a camera frame object.
            
            (Used after grabing a frame with a camera object)
        """
        self.frame = frame

    def compute_bbox(self):
        """
            - Calculates the bounding box of the target object from a
            previously set frame.

            - Sets the bounding box to the bbox attribute.

            (Used after setting the frame )
        """
        self.bbox = self.darknet_detector.inference(self.frame)
        if self.bbox is not None: print("Object detection works.")
    
    def compute_affine(self):    
        """
            - Calculates the 6D pose estimation of the target object, 
            using PVN. Based on the previously set bbox.

            - The 6D pose estimation is computed as a numpy affine 
            matrix.

            - Sets the affine matrix to the affine attribute.

            (Used after setting bbox)
        """    
        _use_icp = False
        if self.bbox is not None:
            detected, affine = self.pvn_detector.inference(self.bbox, self.frame.rgb.copy(
                ), self.frame.depth, self.frame.intrinsic, use_icp=_use_icp)  # affine.shape = 3x4 !
            print("Pose estimation works.")
            self.affine = affine
            self.detected = detected
        else:
            self.affine = None
            self.detected = None
    
    def get_affine(self):
        """
            - Retrieves the affine matrix from the affine atribute.
            
            (Used after setting affine)
        """
        if self.detected:
            return self.affine
        else:
            print("No object was detected")
            return None
