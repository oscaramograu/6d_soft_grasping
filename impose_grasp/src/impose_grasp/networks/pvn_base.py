import numpy as np
import tensorflow as tf
import sys

class PvnSetterAndGetter():
    def __init__(self, paths) -> None:
        sys.path.append(paths["6IMPOSE"])       
        self._paths = paths

        self._mesh_kpts = None
        self._intrinsic_matrix = None
        self._n_sample_point = None
        self._mesh_points = None
        self._pvn3d_model = None
        self._initial_pose_model = None

        self._rgb = None
        self._dpt = None

        self._bbox = None
        self._resnet_input_size = None

# ========================= getter functions =========================
    def get_paths(self):
        """
        Retuns the paths in the following order:
            - mesh
            - kpts
            - corner
            - config
            - weights            
        """
        mesh = self._paths["mesh"]
        kpts = self._paths["kpts"]
        corner = self._paths["corner"]
        config = self._paths["config"]
        weights = self._paths["weights"]

        return mesh, kpts, corner, config, weights

    def get_params_and_data_config(self, config_path, weights_path):
        from utils import read_config
        from lib.data.linemod.linemod_settings import LineModSettings
 
        params = read_config(config_path)
        params.monitor_params.weights_path = weights_path

        data_config = LineModSettings(params.dataset_params.data_name,
                                    params.dataset_params.cls_type,
                                    params.dataset_params.use_preprocessed,
                                    params.dataset_params.crop_image)
        return params, data_config
    
# ========================= setter functions ========================= 
    def set_mesh_kpts(self, kpts_path, corner_path):
        key_points = np.loadtxt(kpts_path)
        center = [np.loadtxt(corner_path).mean(0)]
        mesh_kpts = np.concatenate([key_points, center], axis=0)
        self._mesh_kpts = tf.cast(tf.expand_dims(mesh_kpts, axis=0), dtype=tf.float32)

    def set_intrinsic_matrix(self, data_config):
        self._intrinsic_matrix = data_config.intrinsic_matrix

    def set_n_sample_pts(self, params):
        self._n_sample_points = params.pvn3d_params.point_net2_params.n_sample_points
    
    def set_mesh_points(self, mesh_path, rescale_factor):
        from lib.data.utils import load_mesh
        self._mesh_points = load_mesh(mesh_path, scale=rescale_factor, n_points=500)

    def set_pvn3d_model(self, params, data_config):
        from lib.net.pvn3d_adp import Pvn3dNet        
        self._pvn3d_model = Pvn3dNet(params.pvn3d_params,
                            rgb_input_shape=[80, 80, 3],
                            num_kpts=data_config.n_key_points,
                            num_cls=data_config.n_classes,
                            num_cpts=data_config.n_ctr_points,
                            dim_xyz=data_config.dim_pcld_xyz)
        
        if params.monitor_params.weights_path is not None:
            self._pvn3d_model.load_weights(params.monitor_params.weights_path)
    
    def set_initial_pose_model(self):
        from lib.net.pprocessnet import InitialPoseModel
        self._initial_pose_model = InitialPoseModel()

    def set_rgb_and_dpt(self, rgb, dpt):
        self._rgb = rgb
        self._dpt = dpt

    def set_bbox_and_resnet_input_size(self, bbox, resnet_input_size):
        self._bbox =  bbox
        self._resnet_input_size = resnet_input_size