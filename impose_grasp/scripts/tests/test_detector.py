import os
from PIL import Image
import numpy as np

from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP
from impose_grasp.networks.detector_new import PositionDetector
from impose_grasp.models.cameras.base_camera import CamFrame


rgb_path = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "images", "duck2.jpg")
with Image.open(rgb_path) as rgb:
    rgb = np.array(rgb).astype(np.uint8)

frame = CamFrame
frame.rgb = rgb
frame.depth = None

pd = PositionDetector("cpsduck")
bbox = pd.darknet_detector.inference(frame)

print("initial img size: ", rgb.size)
print("initial img shape: ", rgb.shape)

print("bbox: ", bbox)

cropped_rgb = rgb[bbox[1]:bbox[3], bbox[0]:bbox[2]]

print("cropped img size: ", cropped_rgb.size)
print("cropped img shape: ", cropped_rgb.shape)

cropped_img = Image.fromarray(cropped_rgb)
cropped_img.show()