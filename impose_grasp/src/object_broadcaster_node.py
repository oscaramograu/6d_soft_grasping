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
        while not rospy.is_shutdown() and not self.stop_flag:
            self.object_broadcaster.broadcast_obj_tf()

if __name__ == "__main__":
    target_object_name = "cpsduck"  # Replace with the name of your target object
    object_broadcaster_node = ObjectBroadcasterNode(target_object_name)
    object_broadcaster_node.run()
