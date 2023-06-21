#!/usr/bin/env python

from PIL import Image
import numpy as np

from impose_grasp.nodes.Rt_publisher import RtPublisher
from impose_grasp.nodes.node_utils import path_to_demo_file

if __name__ == '__main__':
    Rt_publisher = RtPublisher(1)
    
    # =============== Real ===============
    while True:
        Rt_publisher.publish_Rt()

    # =============== Test ===============
    # while True:
    #     rgb_path = path_to_demo_file("rgb_0001.png")
    #     dpt_path = path_to_demo_file("dpt_0001.png")

    #     with Image.open(rgb_path) as rgb:
    #         rgb = np.array(rgb).astype(np.uint8)

    #     with Image.open(dpt_path) as depth:
    #         dpt = np.array(depth) / 1000.

    #     Rt_publisher.test_publish_Rt(rgb, dpt)
