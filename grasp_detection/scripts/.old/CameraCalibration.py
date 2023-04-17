import cv2
import numpy as np
import yaml
from yaml.loader import SafeLoader

class ImageUnidistorter():
    def __init__(self, path: str):
        self._cam_mattrix = None
        self._dist_coeffs = None

        self._load_params(path)
        self._print_params()

        
    def _load_params(self, path):
        with open(path) as f:
            data = yaml.load(f, Loader=SafeLoader)
    
            self.cam_mat = np.matrix([[data['fx'], 0, data['cx']],
                                        [0, data['fy'], data['cy']],
                                        [0, 0, 1]])
            
            self.dist_coeffs = np.matrix([
                data['k1'], data['k2'], data['p1'], data['p2'], data['k3']
                 ])
            
    def _print_params(self):
        print("camera matrix: \n", self.cam_mat)
        print("distortion coeffitiens = ", self.dist_coeffs)
            
    def undistort_img(self, img):
        h,  w = img.shape[:2]

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.cam_mat, self.dist_coeffs, (w,h), 1, (w,h))

        dst = cv2.undistort(img, self.cam_mat, self.dist_coeffs)#, None, newcameramtx)

        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        return(dst)

# path = "../../config/l_calib_params.yaml"
# CC = CameraCalibrator(path)