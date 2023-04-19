import cv2
import numpy as np
import yaml
from yaml.loader import SafeLoader


class ImageLoader():
    def __init__(self, camera_index: int = 2) -> None:
        try:
            cap = cv2.VideoCapture(camera_index)
            if cap is None or not cap.isOpened():
                raise ConnectionError
            else:
                self._cap = cap
                self._set_resolution()

        except ConnectionError:
            print("Could not initialize video caption with index: "
                "{}, try with a different index.".format(camera_index))
            self._cap = None

    def _set_resolution(self):  # FHD resolution
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        self._cap.set(3, 1920)
        self._cap.set(4, 1080)
        self._cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

        # self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 64)
        # self.cap.set(cv2.CAP_PROP_CONTRAST, 0)

    def __call__(self):
        ret, frame = self._cap.read()
        try:
            if ret:
                return frame
            else:
                raise ConnectionError
        
        except ConnectionError:
            print("The frame was not captured")

    def __del__(self):
        if self._cap is not None:
            self._cap.release()
            

class ImageCalibrator():
    def __init__(self, path: str):
        self._cam_mattrix = None
        self._dist_coeffs = None

        self._load_params(path)
        self._print_params()

        
    def _load_params(self, path):
        with open(path) as f:
            data = yaml.load(f, Loader=SafeLoader)
    
            self._cam_mat = np.matrix([[data['fx'], 0, data['cx']],
                                        [0, data['fy'], data['cy']],
                                        [0, 0, 1]])
            
            self._dist_coeffs = np.matrix([
                data['k1'], data['k2'], data['p1'], data['p2'], data['k3']
                 ])
            
    def _print_params(self):
        print("camera matrix: \n", self._cam_mat)
        print("distortion coeffitiens = ", self._dist_coeffs)
            
    def __call__(self, img):
        h,  w = img.shape[:2]
        c_params  = (self._cam_mat, self._dist_coeffs)

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(*c_params, (w,h), 1, (w,h))

        calib_img = cv2.undistort(img, *c_params, None, newcameramtx)

        x, y, w, h = roi
        calib_img = calib_img[y:y+h, x:x+w]

        return(calib_img)
    
# IL = ImageLoader(2)
# img = IL()

# print(type(img))