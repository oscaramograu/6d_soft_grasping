#!/usr/bin/env python  

import rospy
from std_msgs.msg import Bool  # Import the ROS message type for the stop broadcasting flag
from impose_grasp.nodes.object_detection.object_broadcaster import ObjectBroadcaster  # Replace "your_package" with the actual package name containing ObjectBroadcaster

class ObjectBroadcasterNode:
    def __init__(self, target_obj):
        # rospy.init_node("object_broadcaster_node")
        self.object_broadcaster = ObjectBroadcaster(target_obj)
        self.stop_flag = False

        # Define the ROS subscriber to receive the stop broadcasting flag
        rospy.Subscriber('stop_broadcasting', Bool, self.stop_broadcasting_callback)

    def stop_broadcasting_callback(self, msg):
        # If the received message has data set to True, stop broadcasting
        if msg.data:
            rospy.loginfo("Received stop broadcasting flag. Stopping broadcasting.")
            self.stop_flag = True

    def run(self):
        n=0
        while not rospy.is_shutdown():
            self.object_broadcaster.broadcast_obj_tf(self.stop_flag)
            if n < 20:
                print("object has been fixed")

                n+=1
            if(n==20):
                print("object has been fixed")
                self.stop_flag == True


if __name__ == "__main__":

    target_obj = "cpsduck"  # Replace with the name of your target object
    object_br = ObjectBroadcaster(target_obj)

    n=0
    stop_flag = False

    while not rospy.is_shutdown():
        object_br.broadcast_obj_tf(stop_flag)
        if n < 20:
            print("object has been found")

            n+=1
        if(n==20):
            print("object has been fixed")
            stop_flag == True