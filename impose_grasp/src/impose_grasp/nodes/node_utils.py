import os

from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP, SIX_IMPOSE_PATH
from impose_grasp.networks.detector import PositionDetector

def path_to_demo_file(path_from_demo_to_file=""):
    path_to_demo = os.path.join(PATH_TO_IMPOSE_GRASP, "data/demo_data")
    return os.path.join(path_to_demo, path_from_demo_to_file) 

def create_detector() -> PositionDetector:
    """ 
    Loads the paths of the files that are necessary to create the darknet detector 
    and for the pvn pose estimator.
    - Darknet: 
        - yolo config
        - obj data
        - yolo pre-trained weights
    - PVN: 
        - object mesh
        - keypoints
        - corners
        - pvn config
        - pvn pre-trained weights    
    """

    darknet_paths = {
        "6IMPOSE": SIX_IMPOSE_PATH,
        "yolo_config": path_to_demo_file("yolo_model/yolov4-tiny-lm-cat.cfg"),
        "data_file": path_to_demo_file("yolo_model/single_obj.data"),
        "yolo_weights": path_to_demo_file("yolo_model/yolov4-tiny-lm-cat_best.weights")
    }

    pvn_paths = {
        "6IMPOSE": SIX_IMPOSE_PATH,
        "mesh": path_to_demo_file("obj_06.ply"),
        "kpts": path_to_demo_file("farthest.txt"),
        "corner": path_to_demo_file("corners.txt"),
        "config": path_to_demo_file("pvn_model/log/cat/config.json"),
        "weights": path_to_demo_file("pvn_model/model/cat/best_model/model"),
        "demo_data": path_to_demo_file()
    }

    return PositionDetector(darknet_paths, pvn_paths)