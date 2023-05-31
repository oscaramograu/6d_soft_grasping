import numpy as np

class DarknetDetector():
    def __init__(self, darknet_paths) -> None:
        self.paths = darknet_paths

        self.network = None
        self.class_names = None
        self.class_colors = None

        self.yolo_thresh = None

        self.width = None
        self.height = None

        self.yolo_rescale_factor = None
        self.dw = None
        self.dh = None

        self.darknet_image = None

        self.resnet_input_size = None
        self.bbox_default = None # hacking 80x80, this part will be done using yolo in the robot

        self.bbox = None

        self.setup_darknet()


    def setup_darknet(self):
        import sys
        sys.path.append(self.paths["6IMPOSE"])
        from darknet import darknet
        from lib.data.utils import formatting_predictions, get_yolo_rescale_values

        self.network, self.class_names, self.class_colors = darknet.load_network(
            self.paths["yolo_config"],
            self.paths["data_file"],
            self.paths["yolo_weights"],
            batch_size=1
        )

        self.yolo_thresh = 0.25

        self.width = darknet.network_width(self.network)
        self.height = darknet.network_height(self.network)

        self.yolo_rescale_factor, self.dw, self.dh = get_yolo_rescale_values()
        self.darknet_image = darknet.make_image(self.width, self.height, 3)

        resnet_w_h = 80
        self.resnet_input_size = [resnet_w_h, resnet_w_h]
        self.bbox_default = [320., 120., 400., 200.]  # hacking 80x80, this part will be done using yolo in the robot

        return True

    def inference(self, rgb):
        import sys
        sys.path.append(self.paths["6IMPOSE"])
        from lib.data.utils import rescale_image_bbox, formatting_predictions
        from darknet import darknet

        bbox2det = lambda bbox: {'coor': np.array(bbox[:4]), 'conf': np.array(bbox[4]), 'image_index': 0}
        image_resized = rescale_image_bbox(np.copy(rgb), (self.width, self.height))
        image_resized = image_resized.astype(np.uint8)
        darknet.copy_image_from_bytes(self.darknet_image, image_resized.tobytes())
        detections = darknet.detect_image(self.network, self.class_names, self.darknet_image, thresh=self.yolo_thresh)
        # darknet.free_image(self.darknet_image)
        pred_bboxes = []

        if len(detections) != 0:
            detect = detections[-1]  # picking the detection with highest confidence score
            bbox = formatting_predictions(detect, self.yolo_rescale_factor, self.dw, self.dh)
            pred_bboxes.extend([bbox2det(box) for box in [bbox]])
            self.bbox = bbox

            return True

        else:
            self.bbox = self.bbox_default
            return False

    def get_resnet_inputs(self):
        return self.bbox, self.resnet_input_size