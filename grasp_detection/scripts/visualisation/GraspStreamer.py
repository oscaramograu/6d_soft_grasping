from old.DepthCalculator import DepthCalculator
from NN.GraspDetector import GraspDetector
import cv2
import numpy as np

class GraspStreamer(DepthCalculator):
    def __init__(self):
        DepthCalculator.__init__(self)
        self.GD = GraspDetector()

        self.grasp_img = None

    def stream_grasp(self):
        while(True):
            self.get_grasp()

            grasp_img = self.grasp_img

            heatmap = self.GD.Q
            normalized_heatmap = (heatmap - np.min(heatmap)) / (np.max(heatmap) - np.min(heatmap))
            normalized_heatmap = cv2.normalize(normalized_heatmap, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)

            normalized_heatmap = cv2.applyColorMap(normalized_heatmap, cv2.COLORMAP_JET)
            cv2.imshow("Grasp Image", grasp_img)
            cv2.imshow("Heatmap", normalized_heatmap)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()
    
    def get_grasp(self):
        self.get_images()
        self.compute_disparity_img()
        self.grasp_img = self.GD.get_grasp_img(self.l_img, self.disparity_img)

GS = GraspStreamer()
GS.stream_grasp()