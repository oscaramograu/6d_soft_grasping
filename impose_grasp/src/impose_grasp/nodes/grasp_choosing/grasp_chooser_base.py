import os
import json
import open3d as o3d
from ros_numpy.geometry import pose_to_numpy

from geometry_msgs.msg import Pose
from impose_grasp.nodes.grasp_choosing.tf_pose_listener import TfPoseListener
from impose_grasp.models.cameras.base_camera import CamFrame
from impose_grasp.nodes.grasp_choosing.frame_builder import FrameBuilder
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP, load_mesh

MODELS_PATH = os.path.join(PATH_TO_IMPOSE_GRASP,
            "data", "models")

class GraspChooserBase(TfPoseListener):
    def __init__(self, target_obj_name):
        super().__init__(target_obj_name)

        self._grasps_path = os.path.join(MODELS_PATH,
            target_obj_name, "gripping_poses.json")
        self._EEF_mesh_path = os.path.join(MODELS_PATH,
            "gripper_collision_d415.stl")

        self._build_atributes()


    def _build_atributes(self):
        """
        Builds all the attributes that are necessary to choose the best grasp.
        """
        self._get_pose_from_tf()
        self._build_object_and_griper_poses(
            self._cam_pose, self._obj_pose)

        self._build_gripping_poses_and_offsets()

        fb = FrameBuilder()
        # self._build_obstruction_pcl(frame)

        # self._build_scene()

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

    def _build_gripping_poses_and_offsets(self):
        """
        builds the lists of possible gripping poses of the target object
        """
        self.grasp_offsets = []
        self.gripping_poses = []

        if os.path.isfile(self._grasps_path):
            with open(self._grasps_path) as F:
                json_load = json.load(F)
                for x in json_load:
                    self.grasp_offsets.append(x["width"])
                    self.gripping_poses.append(x["pose"])
                # grasps = [GrippingPose(eval(
                #     'np.array(' + x["pose"] + ')'), x["width"])
                #     for x in json_load]

    def _build_object_and_griper_poses(self, 
                    camera_pose: Pose, target_pose: Pose):
        """
        Builds the positions of the gripper pose (camera tf frame)
        and the object pose (object tf frame)
        """
        self.gripper_pose = pose_to_numpy(camera_pose)
        self.obj_pose = pose_to_numpy(target_pose)

    def _build_scene(self):
        """
        Builds the mesh of the projection of the end effector.
        """
        gripper_bbox = load_mesh(
        self._EEF_mesh_path, tensor=True)
        scene = o3d.t.geometry.RaycastingScene()
        _ = scene.add_triangles(gripper_bbox)