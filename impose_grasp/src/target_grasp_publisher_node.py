#!/usr/bin/env python  

from impose_grasp.nodes.grasp_choosing.grasp_chooser import GraspChooser


if __name__ == "__main__":
    gr_chooser = GraspChooser("cps_duck")
    grasp_pose = gr_chooser.compute_best_grasp_pose()
    