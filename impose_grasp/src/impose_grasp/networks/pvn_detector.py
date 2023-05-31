import numpy as np
import tensorflow as tf
from PIL import Image
import os

class PvnDetector():
    def __init__(self, paths):
        self.paths = paths 
        
        self.rescale_factor = 0.001
        self.mesh_kpts = None
        self.intrinsic_matrix = None
        self.n_sample_points = None

        self.input_data = None
        self.pvn3d_model = None
        self.initial_pose_model = None

        self.mesh_points = None

        self.setup()

        self.Rt_pre = None
        self.crop_index = None


    def setup(self):
        import sys
        sys.path.append(self.paths["6IMPOSE"])
        from lib.data.utils import load_mesh
        from lib.data.linemod.linemod_settings import LineModSettings
        from utils import read_config
        from lib.net.pvn3d_adp import Pvn3dNet
        from lib.net.pprocessnet import InitialPoseModel

        rescale_factor = self.rescale_factor
        mesh_path = self.paths["mesh"]
        kpts_path = self.paths["kpts"]
        corner_path = self.paths["corner"]
        params = read_config(self.paths["config"])
        params.monitor_params.weights_path = self.paths["weights"]

        data_config = LineModSettings(params.dataset_params.data_name,
                                    params.dataset_params.cls_type,
                                    params.dataset_params.use_preprocessed,
                                    params.dataset_params.crop_image)

        self.mesh_points = load_mesh(mesh_path, scale=rescale_factor, n_points=500)

        key_points = np.loadtxt(kpts_path)
        center = [np.loadtxt(corner_path).mean(0)]
        mesh_kpts = np.concatenate([key_points, center], axis=0)
        self.mesh_kpts = tf.cast(tf.expand_dims(mesh_kpts, axis=0), dtype=tf.float32)
        self.intrinsic_matrix = data_config.intrinsic_matrix
        self.n_sample_points = params.pvn3d_params.point_net2_params.n_sample_points

        self.pvn3d_model = Pvn3dNet(params.pvn3d_params,
                            rgb_input_shape=[80, 80, 3],
                            num_kpts=data_config.n_key_points,
                            num_cls=data_config.n_classes,
                            num_cpts=data_config.n_ctr_points,
                            dim_xyz=data_config.dim_pcld_xyz)

        self.initial_pose_model = InitialPoseModel()

        if params.monitor_params.weights_path is not None:
            self.pvn3d_model.load_weights(params.monitor_params.weights_path)

    def load_imgs(self, rgb, dpt):
        self.rgb = rgb
        self.dpt = dpt

    def load_inputs(self, bbox, resnet_input_size):
        self.bbox =  bbox
        self.resnet_input_size = resnet_input_size

    def input_preparation(self):
        from lib.data.utils import get_crop_index, crop_image, pcld_processor_tf, expand_dim

        crop_index, crop_factor = get_crop_index(self.bbox, base_crop_resolution=self.resnet_input_size)
        self.crop_index = crop_index
        self.rgb = crop_image(self.rgb, crop_index.astype(int))
        depth = crop_image(self.dpt, crop_index.astype(int))
        
        rgb_normalized = self.rgb.copy() / 255.
        pcld_processor_tf(depth.astype(np.float32),
                            rgb_normalized.astype(np.float32), self.intrinsic_matrix, 1,
                            self.n_sample_points, xy_ofst=crop_index[:2],
                            depth_trunc=2.0)
        pcld_xyz, pcld_feats, sampled_index = pcld_processor_tf(depth.astype(np.float32),
                                                                rgb_normalized.astype(np.float32), self.intrinsic_matrix, 1,
                                                                self.n_sample_points, xy_ofst=crop_index[:2],
                                                                depth_trunc=2.0)

        self.rgb = tf.image.resize(self.rgb, self.resnet_input_size).numpy()
        self.input_data = expand_dim(self.rgb, pcld_xyz, pcld_feats, sampled_index, crop_factor)

    def inference(self, rgb, dpt):
        from lib.net.pvn3d_adp import forward_pass
        self.load_imgs(rgb, dpt)
        self.input_preparation()

        kp_pre_ofst, seg_pre, cp_pre_ofst = forward_pass(self.input_data, self.pvn3d_model, training=False)
        R, t, kpts_voted = self.initial_pose_model([self.input_data[1], kp_pre_ofst, cp_pre_ofst, seg_pre, self.mesh_kpts], training=False)
        self.Rt_pre = tf.concat([R[0], tf.expand_dims(t[0], -1)], axis=-1).numpy()

    def rgb_img_for_visualization(self):
        from lib.monitor.visualizer import project_p3d, draw_p2ds
        import cv2
        pred_pts = np.dot(self.mesh_points.copy(), self.Rt_pre[:, :3].T) + self.Rt_pre[:, 3]
        pre_mesh_projected = project_p3d(pred_pts, cam_scale=1, K=self.intrinsic_matrix)
        pre_mesh_projected[:, 0] -= self.crop_index[0]
        pre_mesh_projected[:, 1] -= self.crop_index[1]
        img_pre_projected = draw_p2ds(self.rgb, pre_mesh_projected, r=1, color=[0, 0, 255])

        return img_pre_projected