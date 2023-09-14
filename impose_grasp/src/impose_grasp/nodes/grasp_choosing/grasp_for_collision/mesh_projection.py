import rospy
import numpy as np
import ros_numpy
import cv2
import os
from sensor_msgs.msg import Image

from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP, load_mesh
from impose_grasp.lib.frame_builder import FrameBuilder
from impose_grasp.lib.tf_listener import TfListener

class MeshLoader:
    mesh_pts: np.ndarray

    def load_mesh(self, object_name):
        mesh_path = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "models", 
                            object_name, object_name + ".ply")
        self.mesh_pts = np.asarray(load_mesh(mesh_path).vertices)

class MeshUtils(MeshLoader):
    def transform_mesh(self, Rt)->np.ndarray:
        Rt = Rt.copy()[:3, :]
        return np.dot(self.mesh_pts.copy(), Rt[:, :3].T) + Rt[:, 3]

class Object2ImgProjecter(MeshLoader):
    camera_intrinsic: np.ndarray
    color: np.ndarray
    xy_ofst: tuple
    cam_scale: float

    def __init__(self, intrinsic, color = (255, 0, 0), cam_scale = 1.0, xy_ofst = (0, 0), object_name = "stapler") -> None:
        self.load_mesh(object_name)
        self.color = color
        self.cam_scale = cam_scale
        self.xy_ofst = xy_ofst
        self.camera_intrinsic = intrinsic

    def project2img(self, Rt, img):
        x1, y1 = self.xy_ofst
        mesh_pts_trans = np.dot(self.mesh_pts.copy(), Rt[:, :3].T) + Rt[:, 3]
        mesh_pt2ds = self._project_p3d(p3d=mesh_pts_trans)

        mesh_pt2ds[:, 0] -= x1
        mesh_pt2ds[:, 1] -= y1
        img = self._draw_p2ds(img, p2ds=mesh_pt2ds, r=1) / 255.
        return img

    def _project_p3d(self, p3d):
        K = self.camera_intrinsic
        p3d = p3d * self.cam_scale
        p2d = np.dot(p3d, K.T)
        p2d_3 = p2d[:, 2]
        p2d_3[np.where(p2d_3 < 1e-8)] = 1.0
        p2d[:, 2] = p2d_3
        p2d = np.around((p2d[:, :2] / p2d[:, 2:])).astype(np.int32)  # this is in pixel
        return p2d

    def _draw_p2ds(self, img, p2ds, r=10, ):
        h, w = img.shape[0], img.shape[1]
        for pt_2d in p2ds:
            pt_2d[0] = np.clip(pt_2d[0], 0, w)
            pt_2d[1] = np.clip(pt_2d[1], 0, h)
            img = cv2.circle(
                img, (pt_2d[0], pt_2d[1]), r, self.color, -1
            )
        return img

class ProjectionPublisher(Object2ImgProjecter):
    raw_img: np.ndarray
    Rt: np.ndarray
    drawn_img: np.ndarray

    drawn_img_pub: rospy.Publisher
    frame_builder: FrameBuilder
    cam_obj_list: TfListener

    def __init__(self, object_name="stapler") -> None:
        self.drawn_img_pub = rospy.Publisher("/drawn_img/rgb/img", Image, queue_size=10)
        self.frame_builder = FrameBuilder()
        self.cam_obj_list = TfListener(object_name + "_frame", "camera_frame")
        
        super().__init__(self.frame_builder.frame.intrinsic, (255, 0, 0), 1.0, (0, 0), object_name)

    def _set_raw_img(self):
        frame = self.frame_builder.get_actual_frame()
        self.raw_img = frame.rgb

    def _set_Rt(self):
        self.cam_obj_list.listen()
        Rt_full = self.cam_obj_list.get_np_frame()
        self.Rt = Rt_full[:3,:]

    def draw_img(self):
        self._set_raw_img()
        self._set_Rt()

        self.drawn_img = self.project2img(self.Rt, self.raw_img)*255

    def publish_drawn_img(self):
        img_msg = Image
        img_msg = ros_numpy.msgify(Image, self.drawn_img.astype(np.uint8), "rgb8")
        self.drawn_img_pub.publish(img_msg)