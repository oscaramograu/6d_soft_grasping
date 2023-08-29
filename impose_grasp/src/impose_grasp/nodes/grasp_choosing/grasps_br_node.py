import numpy as np
import random
from impose_grasp.lib.tf_listener import TfListener

from impose_grasp.nodes.grasp_choosing.grasp_processing.grasps_orienter import GraspOrienter
from impose_grasp.nodes.grasp_choosing.grasp_processing.grasps_mapper import GraspMapper
from impose_grasp.nodes.grasp_choosing.grasp_processing.grasp_chooser import GraspChooser
from impose_grasp.nodes.grasp_choosing.grasp_processing.grasp_filterer import GraspFilterer
from impose_grasp.nodes.grasp_choosing.grasp_processing.grasps_broadcaster import GraspsBroadcasater

class GraspBrNode:
    def __init__(self, obj_name) -> None:
        self._obj_tf_listener = TfListener(obj_name + "_frame")
        self._cam_tf_listener = TfListener("camera_frame")
        
        chooser = self._build_chooser()
        self.target = chooser.compute_best_grasp_ind()
        # good_grasp_ids = [i for i in range(len(chooser.rel_poses)) 
        #                   if chooser.good_gr_flags[i]]  
        # self.target = good_grasp_ids[random.randrange(0, len(good_grasp_ids))]
        # self.target = 9      
        print("The target grasp is: ", self.target)
        self.broadcaster = GraspsBroadcasater(chooser)

    def __call__(self, br_only_target: bool):
        self.broadcaster.broadcast_target(self.target)

        if not br_only_target:
            self.broadcaster.broadcast_good_grasps()
      
    def _build_chooser(self):
        obj_pose, cam_pose = self._get_and_listen_poses()

        orienter = GraspOrienter(obj_pose)
        orienter.orient_grasps()

        mapper = GraspMapper(width_th=0.04, theta=-30, grasps=orienter)
        mapper.map_grasps(using_offset=True)
    

        filterer = GraspFilterer(mapper)
        filterer.filter(obj_pose, cam_pose)
        
        return GraspChooser(filterer)
    
    def _get_and_listen_poses(self)-> (np.ndarray, np.ndarray):
        self._obj_tf_listener.listen_tf()
        obj_pose = self._obj_tf_listener.get_np_frame()

        self._cam_tf_listener.listen_tf()
        cam_pose = self._cam_tf_listener.get_np_frame()

        return obj_pose, cam_pose
    