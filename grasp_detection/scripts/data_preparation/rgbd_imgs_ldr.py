import cv2
import numpy as np
import data_preparation.UVC_img_ldr as uvc 

class StereoImgsDictLoader(uvc.ImageLoader):
    def __init__(self, camera_index: int = 2) -> None: # Load camera index from yaml file
        super().__init__(camera_index)
        self.imgs = {
            "rgb_l": None,
            "rgb_r": None,
            "gs_l": None,
            "gs_r": None
        }
        self._img_cal_l = uvc.ImageCalibrator("config/l_calib_params.yaml")
        self._img_cal_r = uvc.ImageCalibrator("config/r_calib_params.yaml")


    def _compute_rgb_stereo_images(self): 
        merged_images_frame = self.get_rgb_image()

        h, w = merged_images_frame.shape[0], merged_images_frame.shape[1]//2

        img_l, img_r = merged_images_frame[:, :w,:], merged_images_frame[:, w:,:]

        img_l , img_r = cv2.resize(img_l, (w//2, h//2)), cv2.resize(img_r, (w//2, h//2))

        img_l , img_r = self._img_cal_l(img_l),  self._img_cal_r(img_r)

        self.imgs["rgb_l"], self.imgs["rgb_r"] =  img_l , img_r
    
    def _compute_grayscale_imgs(self):
        self.imgs["gs_l"] = cv2.cvtColor(self.imgs["rgb_l"],cv2.COLOR_BGR2GRAY)
        self.imgs["gs_r"] = cv2.cvtColor(self.imgs["rgb_r"],cv2.COLOR_BGR2GRAY)
    
    def __call__(self) -> dict:
        self._compute_rgb_stereo_images()
        self._compute_grayscale_imgs()
        return self.imgs


class DepthImgDictLoader():
    def __init__(self) -> None:
        self._img_ldr = StereoImgsDictLoader()

        self.imgs = {
            "disp_map": None,
            "depth": None
        }

        self._disp_params = {  # Load them from a YAML file
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

    def __call__(self, gs_l, gs_r):
        self._compute_disparity(gs_l, gs_r)
        self._compute_depth()
        return self.imgs

class ImgsDictUpdater():
    def __init__(self) -> None:
        self.imgs = {
            "rgb_l": None,
            "rgb_r": None,
            "gs_l": None,
            "gs_r": None,
            "disp_map": None,
            "depth": None                        
        }

        self._rgb_gs_ldr = StereoImgsDictLoader()
        self._depth_lr = DepthImgDictLoader()

    def __call__(self):
        rgb_gs_imgs = self._rgb_gs_ldr()

        disp_depth_imgs = self._depth_lr(rgb_gs_imgs["gs_l"], rgb_gs_imgs["gs_r"])

        self.imgs.update(rgb_gs_imgs)
        self.imgs.update(disp_depth_imgs)

    def __getitem__(self, key: str):
        return self.imgs[key]