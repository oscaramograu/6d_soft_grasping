#!/usr/bin/env python  
from datetime import datetime
import rospy
from std_msgs.msg import Bool  # Import the ROS message type for the stop broadcasting flag
from impose_grasp.nodes.object_detection.object_broadcaster import ObjectBroadcaster  # Replace "your_package" with the actual package name containing ObjectBroadcaster


if __name__ == "__main__":

    rospy.init_node("object_broadcaster_node")

    object_br = ObjectBroadcaster("cpsduck")

    n=0
    stop_flag = False

    old = datetime.now()
    cycles = 40
    r = rospy.Rate(5) # 10Hz
    while not rospy.is_shutdown():

        # old = act
        object_br.broadcast_obj_tf(stop_flag)
        if n < cycles:
            n+=1
            print("object has been found")

        elif(n==cycles):
            n+=1
            print("object has been fixed")
            stop_flag = True
        else:
            r.sleep()
