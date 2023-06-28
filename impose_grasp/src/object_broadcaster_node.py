#!/usr/bin/env python  

import rospy
from impose_grasp.nodes.object_broadcaster import ObjectBroadcaster

if __name__ == '__main__':
    target_obj = rospy.get_param("/target_obj")

    obj_br = ObjectBroadcaster(target_obj)

    while not rospy.is_shutdown():
        # obj_br.broadcast_obj_tf()
        obj_br.test_broadcaster()

    rospy.spin()