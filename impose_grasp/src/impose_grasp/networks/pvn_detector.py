import numpy as np
import tensorflow as tf
from impose_grasp.networks.pvn_base import PvnSetterAndGetter

class PvnDetector(PvnSetterAndGetter):
    def __init__(self, paths):
        super().__init__(paths)

        self._rescale_factor = 0.001

        self._setup()

        self._crop_index = None
        self._input_data = None

        self.Rt_pre = None


    def _setup(self):
        """ 
        Required setup to perform pvn object detection.

        It sets up the following attributes:
            - key points
            - intrinsic matrix
            - number of sample points
            - mesh points
            - pvn 3d model
            - initial pose model
        """
        mesh_path, kpts_path, corner_path, config_path, weights_path = self.get_paths()
        params, data_config = self.get_params_and_data_config(config_path, weights_path)

        self.set_mesh_kpts(kpts_path, corner_path)
        self.set_intrinsic_matrix(data_config)
        self.set_n_sample_pts(params)
        self.set_mesh_points(mesh_path, self._rescale_factor)
        self.set_pvn3d_model(params, data_config)
        self.set_initial_pose_model()

    def _input_preparation(self):
        """
        Prepares the RGB and Depth images, to match the pvn network inputs.
        The final value that will be sent to PVN is the attribute **input data**
        """
        from lib.data.utils import get_crop_index, crop_image, pcld_processor_tf, expand_dim

        self._crop_index, crop_factor = get_crop_index(self._bbox, base_crop_resolution=self._resnet_input_size)

        self._rgb = crop_image(self._rgb, self._crop_index.astype(int))
        self._dpt = crop_image(self._dpt, self._crop_index.astype(int))
        
        rgb_normalized = self._rgb.copy() / 255.
        pcld_processor_tf(self._dpt.astype(np.float32),
                            rgb_normalized.astype(np.float32), self._intrinsic_matrix, 1,
                            self._n_sample_points, xy_ofst=self._crop_index[:2],
                            depth_trunc=2.0)
        pcld_xyz, pcld_feats, sampled_index = pcld_processor_tf(self._dpt.astype(np.float32),
                                                                rgb_normalized.astype(np.float32), self._intrinsic_matrix, 1,
                                                                self._n_sample_points, xy_ofst=self._crop_index[:2],
                                                                depth_trunc=2.0)

        self._rgb = tf.image.resize(self._rgb, self._resnet_input_size).numpy()

        self._input_data = expand_dim(self._rgb, pcld_xyz, pcld_feats, sampled_index, crop_factor)

    def inference(self, rgb, dpt, bbox, resnet_input_size):
        """
        Executes the inference of the PVN neural network. 
        - It first sets the image (RGB and Depth) as well as the bbox and input size (from Darknet Detector).
        - It then executes a forward pass to PVN.
        - It finally retrieves the rotation and translation, and save it under the attribute 
        **Rt_pre**
        """
        from lib.net.pvn3d_adp import forward_pass
        self.set_rgb_and_dpt(rgb, dpt)
        self.set_bbox_and_resnet_input_size(bbox, resnet_input_size)

        self._input_preparation()

        kp_pre_ofst, seg_pre, cp_pre_ofst = forward_pass(self._input_data, self._pvn3d_model, training=False)
        R, t, kpts_voted = self._initial_pose_model([self._input_data[1], kp_pre_ofst, cp_pre_ofst, seg_pre, self._mesh_kpts], training=False)
        self.Rt_pre = tf.concat([R[0], tf.expand_dims(t[0], -1)], axis=-1).numpy()

    def rgb_img_for_visualization(self):
        from lib.monitor.visualizer import project_p3d, draw_p2ds
        pred_pts = np.dot(self._mesh_points.copy(), self.Rt_pre[:, :3].T) + self.Rt_pre[:, 3]
        pre_mesh_projected = project_p3d(pred_pts, cam_scale=1, K=self._intrinsic_matrix)
        pre_mesh_projected[:, 0] -= self._crop_index[0]
        pre_mesh_projected[:, 1] -= self._crop_index[1]
        img_pre_projected = draw_p2ds(self._rgb, pre_mesh_projected, r=1, color=[0, 0, 255])

        return img_pre_projected