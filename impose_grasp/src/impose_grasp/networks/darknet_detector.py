import numpy as np
from typing import List, Tuple
import sys

class DarknetDetector():
    def __init__(self, darknet_paths) -> None:
        self._paths = darknet_paths
        self._network = None
        self._class_names = None
        self._class_colors = None

        self._yolo_thresh = None

        self._width = None
        self._height = None

        self._yolo_rescale_factor = None
        self._dw = None
        self._dh = None

        self._darknet_image = None

        self._resnet_input_size = None
        self._bbox_default = None # hacking 80x80, this part will be done using yolo in the robot

        self._bbox = None

        self._setup_darknet()


    def _setup_darknet(self) -> bool:
        """ It initializes the darknet object detector """
        sys.path.append(self._paths["6IMPOSE"])
        from darknet import darknet
        from lib.data.utils import get_yolo_rescale_values

        self._network, self._class_names, self._class_colors = darknet.load_network(
            self._paths["yolo_config"],
            self._paths["data_file"],
            self._paths["yolo_weights"],
            batch_size=1
        )

        self._yolo_thresh = 0.25

        self._width = darknet.network_width(self._network)
        self._height = darknet.network_height(self._network)

        self._yolo_rescale_factor, self._dw, self._dh = get_yolo_rescale_values()
        self._darknet_image = darknet.make_image(self._width, self._height, 3)

        resnet_w_h = 80
        self._resnet_input_size = [resnet_w_h, resnet_w_h]
        self._bbox_default = [320., 120., 400., 200.]  # hacking 80x80, this part will be done using yolo in the robot

        return True

    def inference(self, rgb) -> bool:
        """ 
            Executes the inference of the darknet to get the bbox of the object if its detected.
            If its not detected the bbox is set to the default value.
        """
        from darknet import darknet
        from lib.data.utils import rescale_image_bbox, formatting_predictions

        bbox2det = lambda bbox: {'coor': np.array(bbox[:4]), 'conf': np.array(bbox[4]), 'image_index': 0}
        image_resized = rescale_image_bbox(np.copy(rgb), (self._width, self._height))
        image_resized = image_resized.astype(np.uint8)
        darknet.copy_image_from_bytes(self._darknet_image, image_resized.tobytes())
        detections = darknet.detect_image(self._network, self._class_names, self._darknet_image, thresh=self._yolo_thresh)
        # darknet.free_image(self._darknet_image)
        pred_bboxes = []

        if len(detections) != 0:
            detect = detections[-1]  # picking the detection with highest confidence score
            bbox = formatting_predictions(detect, self._yolo_rescale_factor, self._dw, self._dh)
            pred_bboxes.extend([bbox2det(box) for box in [bbox]])
            self._bbox = bbox

            return True

        else:
            self._bbox = self._bbox_default
            return False

    def get_resnet_inputs(self) -> Tuple[list, List[int]]:
        """ Gets the bounding box and the resnet input size """
        return self._bbox, self._resnet_input_size