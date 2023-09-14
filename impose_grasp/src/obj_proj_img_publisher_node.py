#!/usr/bin/env python  

import rospy
from impose_grasp.nodes.grasp_choosing.grasp_for_collision.mesh_projection import ProjectionPublisher

def main():
    rospy.init_node('img_publisher_node', anonymous=True)
    rate = rospy.Rate(10)  # Publish at 10Hz

    obj = rospy.get_param("/target_object")
    proj_pub = ProjectionPublisher(obj)

    while not rospy.is_shutdown():
        proj_pub.draw_img()
        proj_pub.publish_drawn_img()
        rate.sleep()

if __name__ == "__main__":
    main()