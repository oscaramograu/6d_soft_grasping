#!/usr/bin/env python  
import rospy

from impose_grasp.lib.flag import Flag
from impose_grasp.nodes.object_detection.object_broadcaster import ObjectBroadcaster 

if __name__ == "__main__":
    rospy.init_node("object_broadcaster_node")
    restart_flag = Flag()

    obj = rospy.get_param("/target_object")
    object_br = ObjectBroadcaster(obj)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        object_br.broadcast_tf()

        if restart_flag():
            object_br.restart()
            restart_flag.set_flag(False)
        rate.sleep()