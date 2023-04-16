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

class GraspDetector:
    def __init__(self):
        sys.path.append('/home/oscar/catkin_ws/src/thesis/model/ggcnn')

        self.model = torch.load('/home/oscar/catkin_ws/src/thesis/model/ggcnn/ggcnn_weights_cornell/ggcnn_epoch_23_cornell', map_location=torch.device('cpu'))

        self.rgb = None
        self.d = None

        self.rgb_crop = None
        self.d_scaled = None
        self.d_nan = None

        self.Q = None
        self.theta = None
        self.w = None

        self.grasp_img = None

        self.num_clusters = 5

        self.prev_mp = np.array([150, 150])
        self.crop_size = 400

    def load_img(self, rgb, d):
        self.rgb = rgb
        self.d = d 

    def img_processing(self):
        rgb = self.rgb
        depth = self.d

        # Crop a square out of the middle of the depth and resize it to 300*300  
        crop_size = self.crop_size
        self.rgb_crop = cv2.resize(rgb[(480-crop_size)//2:(480-crop_size)//2+crop_size, (640-crop_size)//2:(640-crop_size)//2+crop_size], (300, 300))

        depth_crop = cv2.resize(depth[(480-crop_size)//2:(480-crop_size)//2+crop_size, (640-crop_size)//2:(640-crop_size)//2+crop_size], (300, 300))

        # Replace nan with 0 for inpainting.
        depth_crop = depth_crop.copy()
        self.d_nan = np.isnan(depth_crop).copy()
        depth_crop[self.d_nan ] = 0

        # open cv inpainting does weird things at the border.
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)

        mask = (depth_crop == 0).astype(np.uint8)

        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        depth_scale = np.abs(depth_crop).max()
        depth_crop = depth_crop.astype(np.float32)/depth_scale  # Has to be float32, 64 not supported.

        depth_crop = cv2.inpaint(depth_crop, mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        depth_crop = depth_crop[1:-1, 1:-1]
        depth_crop = depth_crop * depth_scale

        # Run it through the network.
        depth_crop = np.clip((depth_crop - depth_crop.mean()), -1, 1)

        depth_crop = depth_crop.reshape((1, 300, 300))

        self.d_scaled = torch.from_numpy(depth_crop.astype(np.float32))
    
    def use_model(self):
        with torch.no_grad():
            pred_out = self.model.forward(self.d_scaled)

            self.Q = pred_out[0].squeeze()
            self.Q[self.d_nan ] = 0
    
        # Calculate the angle map.
        cos_out = pred_out[1].squeeze()
        sin_out = pred_out[2].squeeze()


        self.theta = np.arctan2(sin_out, cos_out)/2.0

        self.w = pred_out[3].squeeze() * 150.0  # Scaled 0-150:0-1   
    
    def output_filtering(self):
        # Filter the outputs.
        self.Q = ndimage.gaussian_filter(self.Q, 5.0)  # 3.0
        self.theta = ndimage.gaussian_filter(self.theta, 2.0)
    
    def highest_peak_candidate(self):
        maxes = peak_local_max(self.Q, min_distance=10, threshold_abs=0.1, num_peaks=3)

        if maxes.shape[0] == 0:
            return 0
        
        else:
            max_pixel = maxes[np.argmin(np.linalg.norm(maxes - self.prev_mp, axis = 1))]
            self.prev_mp = (max_pixel * 0.25 + self.prev_mp * 0.75).astype(np.int64)

            # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.
            crop_size = self.crop_size
            max_pixel = ((np.array(max_pixel) / 300.0 * crop_size) + np.array([(480 - crop_size)//2, (640 - crop_size) // 2]))
            max_pixel = np.round(max_pixel).astype(np.int64)

            

            # point_depth = self.d[max_pixel[0], max_pixel[1]]

        return 1
    
    def make_grasp_img(self):
        max_pixel = self.prev_mp

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

        self.grasp_img = self.rgb_crop
        cv2.polylines(self.grasp_img,[pts],True,(0,255,255),2)

    def plot_output(self):
        fig, axes = plt.subplots(ncols=4, figsize=(10, 5))

        axes[0].imshow(self.rgb_crop)
        axes[0].set_title('RGB')

        axes[1].imshow(self.d_scaled.squeeze(), cmap='gray')
        axes[1].set_title('Depth')

        axes[2].imshow(self.Q, cmap='jet')
        axes[2].set_title('Grasp Prob Heatmap')

        axes[3].imshow(self.theta, cmap='jet')
        axes[3].set_title('Angle Heatmap')

        plt.show()

    def plot_grasp(self):
        plt.imshow(self.grasp_img)
        plt.show()

    def get_grasp_img(self, rgb, d):
        self.load_img(rgb, d)
        self.img_processing()
        self.use_model()
        self.output_filtering()
        # for i in range(10):
        self.highest_peak_candidate()
        self.make_grasp_img()
        return self.grasp_img

# rgb = cv2.imread("/home/oscar/catkin_ws/src/thesis/model/rgb.png")
# rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
# d = imread("/home/oscar/catkin_ws/src/thesis/model/d.tiff")

# gd = GraspDetector()
# grasp = gd.get_grasp_img(rgb, d)
# plt.imshow(grasp)
# plt.show()