import torch
import torchvision.transforms as transforms
import cv2
import numpy as np
from imageio.v2 import imread
from matplotlib import pyplot as plt
import scipy.ndimage as ndimage
import sys
from sklearn.cluster import KMeans
from skimage.feature import peak_local_max
from data_preparation import rgbd_imgs_ldr

class GraspImgsLoader():
    def __init__(self, model: torch) -> None:
        self._model = model
        self._g_imgs = {
            "Q": None,
            "Theta": None,
            "W": None,
            "grasp": None
        }

        self._params = {
            "n_clusters": 5,
            "d_nan": None,
            "crop_size": 400
        }

        self._d_scaled = None
        self._rgb_scaled = None

    def _input_preprossesing(self, d_img):
        crop_size = self._params["crop_size"]

        d_crop = d_img[(480-crop_size)//2:(480-crop_size)//2+crop_size, (640-crop_size)//2:(640-crop_size)//2+crop_size]
        d_resized = cv2.resize(d_crop, (300, 300))

        # Replace nan with 0 for inpainting.
        d_resized = d_resized.copy()
        d_nan = np.isnan(d_resized).copy()
        self._params["d_nan"] = d_nan
        d_resized[d_nan] = 0

       # open cv inpainting does weird things at the border.
        d_resized = cv2.copyMakeBorder(d_resized, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        mask = (d_resized == 0).astype(np.uint8)

        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        depth_scale = np.abs(d_resized).max()
        d_resized = d_resized.astype(np.float32)/depth_scale  # Has to be float32, 64 not supported.

        d_resized = cv2.inpaint(d_resized, mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        d_resized = d_resized[1:-1, 1:-1]
        d_resized = d_resized * depth_scale

        d_resized = np.clip((d_resized - d_resized.mean()), -1, 1)

        d_resized = d_resized.reshape((1, 300, 300))

        self.d_scaled = torch.from_numpy(d_resized.astype(np.float32))
    
    def _use_model(self, d_scaled):
        with torch.no_grad():
            pred_out = self.model.forward(d_scaled)

            q = pred_out[0].squeeze()
            d_nan = self._params["d_nan"]
            q[d_nan ] = 0
    
        # Calculate the angle map.
        cos_out = pred_out[1].squeeze()
        sin_out = pred_out[2].squeeze()

        theta = np.arctan2(sin_out, cos_out)/2.0

        w = pred_out[3].squeeze() * 150.0  # Scaled 0-150:0-1
        return q, theta, w

    def output_filtering(self, q, theta, w):
        # Filter the outputs.
        q = ndimage.gaussian_filter(q, 5.0)  # 3.0
        theta = ndimage.gaussian_filter(theta, 2.0)
    
        self._g_imgs["Q"] = q
        self._g_imgs["W"] = w
        self._g_imgs["Theta"] = theta

    
    def highest_peak_candidate(self):
        prev_mp = self._params["prev_mp"]
        crop_size = self._params["crop_size"]

        maxes = peak_local_max(q, min_distance=10, threshold_abs=0.1, num_peaks=3)

        max_pixel = maxes[np.argmin(np.linalg.norm(maxes - prev_mp, axis = 1))]
        prev_mp = (max_pixel * 0.25 + prev_mp * 0.75).astype(np.int64)

        # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.
        # max_pixel = ((np.array(max_pixel) / 300.0 * crop_size) + np.array([(480 - crop_size)//2, (640 - crop_size) // 2]))
        max_pixel = np.round(max_pixel).astype(np.int64)

        # point_depth = self.d[max_pixel[0], max_pixel[1]]

        return max_pixel
    
    def make_grasp_img(self, max_pixel, rgb_img):
        crop_size = self._params["crop_size"]

        ang = self.theta[max_pixel[0], max_pixel[1]]
        ang = self.theta.max()

        width = self.w[max_pixel[0], max_pixel[1]]

        center = (int(max_pixel[0]), int(max_pixel[1]))

        # Define the coordinates of the square
        w, h = width, width/2
        x, y = center[1] - w/2, center[0] - h/2

        points = [(x, y), (x+w, y), (x+w, y+h), (x, y+h)]

        M = cv2.getRotationMatrix2D(center, ang, 1)

        # Rotate the square
        rotated_points = [tuple(M.dot((p[0], p[1], 1))[:2]) for p in points]

        # Draw the rotated square onto the image
        pts = np.array(rotated_points, np.int32)
        pts = pts.reshape((-1,1,2))

       # Crop a square out of the middle of the depth and resize it to 300*300  
        rgb_crop = cv2.resize(rgb_img[(480-crop_size)//2:(480-crop_size)//2+crop_size, (640-crop_size)//2:(640-crop_size)//2+crop_size], (300, 300))

        grasp_img = rgb_crop
        cv2.polylines(grasp_img,[pts],True,(0,255,255),2)

        self._g_imgs["grasp_img"] = grasp_img

    def __call__(self, rgb_img, d_img) -> Any:
        self._input_preprossesing(d_img)
        self._use_model()