import os
import open3d as o3d
from ros_numpy.geometry import pose_to_numpy

from impose_grasp.lib.tf_listener import TfListener
from impose_grasp.models.cameras.base_camera import CamFrame
from impose_grasp.lib.frame_builder import FrameBuilder
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP, load_mesh
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps

MODELS_PATH = os.path.join(PATH_TO_IMPOSE_GRASP,
            "data", "models")

class GraspChooserBase(Grasps):
    def __init__(self, grasps: Grasps):
        super().__init__()
        self.set_rel_poses(grasps.rel_poses)
        self.set_widths(grasps.widths)
        self.set_power_gr(grasps.power_gr)

        self._EEF_mesh_path = os.path.join(MODELS_PATH,
            "hand_col.stl")

        self._build_atributes()


    def _build_atributes(self):
        """
        Builds all the attributes that are necessary to choose the best grasp.
        """
        fb = FrameBuilder()
        frame = fb.get_actual_frame()

        self._build_camera_pose()
        self._build_obstruction_pcl(frame)
        self._build_scene()

    def _build_camera_pose(self):
        cam_listener = TfListener("camera_frame")
        cam_listener.listen_tf()
        self.cam_pose = cam_listener.get_np_frame()

    def _build_obstruction_pcl(self, frame: CamFrame):
        """
        Builds the obstruction point cloud from the camera frame. 
        """
        stride_ = 4
        remove_tolerance = 0.01
        persistency = 1.0
        gpu = o3d.core.Device('CUDA:0')

        rgb = o3d.t.geometry.Image(o3d.core.Tensor.from_numpy(
            frame.rgb))#.to(gpu)
        depth = o3d.t.geometry.Image(o3d.core.Tensor.from_numpy(
            frame.depth))#.to(gpu)
        rgbd = o3d.t.geometry.RGBDImage(rgb, depth)
        intr_mat = o3d.core.Tensor(frame.intrinsic)
        pcd = o3d.t.geometry.PointCloud.create_from_rgbd_image(
            rgbd, intr_mat, depth_scale=1.0, stride=max(stride_, 1))

        voxel_size = 0.0
        if voxel_size > 0.0:
            pcd = pcd.voxel_down_sample(voxel_size)

        self.obstruction_pcl = pcd.cpu().clone()

    def _build_scene(self):
        """
        Builds the mesh of the projection of the end effector.
        """
        gripper_bbox = load_mesh(
        self._EEF_mesh_path, tensor=True)
        self.scene = o3d.t.geometry.RaycastingScene()
        _ = self.scene.add_triangles(gripper_bbox)