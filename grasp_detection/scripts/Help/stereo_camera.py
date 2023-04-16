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
            print("Could not initialize video caption with index: {}, try with a different index.".format(camera_index))

    def _set_resolution(self):  # FHD resolution
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        self._cap.set(3, 1920)
        self._cap.set(4, 1080)
        self._cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

        # self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 64)
        # self.cap.set(cv2.CAP_PROP_CONTRAST, 0)

    def get_rgb_image(self):
        ret, frame = self._cap.read()
        try:
            if ret:
                return frame
            else:
                raise ConnectionError
        
        except ConnectionError:
            print("The frame was not captured")

class StereoImgsLoader(ImageLoader):
    def __init__(self, camera_index: int = 2) -> None:
        super().__init__(camera_index)
        self.imgs = {
            "rgb_l": None,
            "rgb_r": None,
            "gs_l": None,
            "gs_r": None
        }

    def _compute_rgb_stereo_images(self): 
        merged_images_frame = self.get_rgb_image()

        h = merged_images_frame.shape[0]
        w = merged_images_frame.shape[1]//2

        l_img, r_img = merged_images_frame[:, :w,:], merged_images_frame[:, w:,:]

        l_img = cv2.resize(l_img, (w//2, h//2))
        r_img = cv2.resize(r_img, (w//2, h//2))

        self.imgs["rgb_l"], self.imgs["rgb_r"] =  l_img, r_img
    
    def _compute_grayscale_imgs(self):
        self.imgs["gs_l"] = cv2.cvtColor(self.imgs["rgb_l"],cv2.COLOR_BGR2GRAY)
        self.imgs["gs_r"] = cv2.cvtColor(self.imgs["rgb_r"],cv2.COLOR_BGR2GRAY)
    
    def compute_rgb_gs_imgs(self) -> dict:
        self._compute_rgb_stereo_images()
        self._compute_grayscale_imgs()
        return self.imgs


class DepthImgLoader():
    def __init__(self) -> None:
        self._img_ldr = StereoImgsLoader()

        self.imgs = {
            "disp_map": None,
            "depth": None
        }

        self._disp_params = {
            "wSize": 11,
            "nDisp": 32,
            "lambda": 75e3,
            "sigma": .5
        }

        self._matcher_l, self._matcher_r = self._stereo_matching_settings()
        self._wls_filter = self._filter_settings()

    def _stereo_matching_settings(self) -> tuple:
        window_size = self._disp_params["wSize"]
        min_disp = 0
        num_disp = self._disp_params["nDisp"]

        matcher_l = cv2.StereoSGBM_create(
            blockSize = 5,
            numDisparities = num_disp,
            minDisparity = min_disp,
            P1 = 8*3*window_size**2,
            P2 = 32*3*window_size**2,
            disp12MaxDiff = 1,
            uniquenessRatio = 15,
            speckleWindowSize = 0,
            speckleRange = 5,
            preFilterCap = 63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
        
        matcher_r = cv2.ximgproc.createRightMatcher(matcher_l)

        return matcher_l, matcher_r

    def _filter_settings(self):
        lmbda = self._disp_params["lambda"]
        sigma = self._disp_params["sigma"]

        wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=self._matcher_l)
        wls_filter.setLambda(lmbda)
        wls_filter.setSigmaColor(sigma)
        return wls_filter

    def _compute_disparity(self, gs_l, gs_r):
        displ = self._matcher_l.compute(gs_l, gs_r).astype(np.int16)
        dispr = self._matcher_r.compute(gs_r, gs_l).astype(np.int16)

        filtered_disp_img = self._wls_filter.filter(displ, gs_l, None, dispr)
        norm_disp_img = cv2.normalize(
            src=filtered_disp_img,
            dst=filtered_disp_img,
            beta=1,
            alpha=255,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_8U
            )

        self.imgs["disp_map"] =  norm_disp_img.astype(np.int16)

    def _compute_depth(self):
        pass

    def compute_disp_depth_imgs(self, gs_l, gs_r):
        self._compute_disparity(gs_l, gs_r)
        self._compute_depth()
        return self.imgs

class ImgsDictCreator():
    def __init__(self) -> None:
        self.imgs = {
            "rgb_l": None,
            "rgb_r": None,
            "gs_l": None,
            "gs_r": None,
            "disp_map": None,
            "depth": None                        
        }

        self._rgb_gs_ldr = StereoImgsLoader()
        self._depth_lr = DepthImgLoader()

    def compute_all_imgs(self):
        rgb_gs_imgs = self._rgb_gs_ldr.compute_rgb_gs_imgs()

        disp_depth_imgs = self._depth_lr.compute_disp_depth_imgs(rgb_gs_imgs["gs_l"], rgb_gs_imgs["gs_r"])

        self.imgs.update(rgb_gs_imgs)
        self.imgs.update(disp_depth_imgs)

    def __getitem__(self, key: str):
        return self.imgs[key]