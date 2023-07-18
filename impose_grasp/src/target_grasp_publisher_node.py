#!/usr/bin/env python  
import rospy
from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser
from impose_grasp.nodes.object_detection.transform_broadcaster import TransformBroadcaster

if __name__ == "__main__":
    rospy.init_node('tf_pose_listener_node')

    obj = "cpsduck"
    obj_frame = "/" + obj + "_frame"
    gr_chooser = GraspChooser(obj)
    grasp_pose = gr_chooser.compute_best_grasp_pose()    
    tf_br = TransformBroadcaster(obj_frame, "target_grasp")
    print(grasp_pose)
    while not rospy.is_shutdown():
        tf_br.broadcast_transform(grasp_pose)