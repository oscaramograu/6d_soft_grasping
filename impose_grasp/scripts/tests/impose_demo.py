from PIL import Image
import numpy as np

from impose_grasp.networks.detector import PositionDetector
from impose_grasp.lib.utils import path_to_demo_file

six_impose_path = "/home/oscar/Desktop/code/6IMPOSE"

darknet_paths = {
    "6IMPOSE": six_impose_path,
    "yolo_config": path_to_demo_file("yolo_model/yolov4-tiny-lm-cat.cfg"),
    "data_file": path_to_demo_file("yolo_model/single_obj.data"),
    "yolo_weights": path_to_demo_file("yolo_model/yolov4-tiny-lm-cat_best.weights")
}

pvn_paths = {
    "6IMPOSE": six_impose_path,
    "mesh": path_to_demo_file("obj_06.ply"),
    "kpts": path_to_demo_file("farthest.txt"),
    "corner": path_to_demo_file("corners.txt"),
    "config": path_to_demo_file("pvn_model/log/cat/config.json"),
    "weights": path_to_demo_file("pvn_model/model/cat/best_model/model"),
    "demo_data": path_to_demo_file()
}

rgb_path = path_to_demo_file("rgb_0001.png")
dpt_path = path_to_demo_file("dpt_0001.png")

with Image.open(rgb_path) as rgb:
    rgb = np.array(rgb).astype(np.uint8)

with Image.open(dpt_path) as depth:
    dpt = np.array(depth) / 1000.

detector = PositionDetector(darknet_paths, pvn_paths)
detector.detect(rgb, dpt)
detector.safe_result()

