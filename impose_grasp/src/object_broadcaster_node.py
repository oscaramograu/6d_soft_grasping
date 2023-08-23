#!/usr/bin/env python  
from datetime import datetime
import rospy
from std_msgs.msg import String  # Import the ROS message type for the stop broadcasting flag
from impose_grasp.nodes.object_detection.object_broadcaster import ObjectBroadcaster  # Replace "your_package" with the actual package name containing ObjectBroadcaster

class Flag:
    def __init__(self) -> None:
        self.flag = False
        rospy.Subscriber("stop_flag", String, self.callback)

    def callback(self, data):
        print("Stop flag has been called")
        self.flag = False


if __name__ == "__main__":
    rospy.init_node("object_broadcaster_node")
    flag = Flag()
    obj = rospy.get_param("/target_object")

    object_br = ObjectBroadcaster(obj)

    n=0
    old = datetime.now()
    cycles = 10
    r = rospy.Rate(2) # 10Hz
    while not rospy.is_shutdown():
        old = datetime.now()
        r.sleep()
        detected = object_br.broadcast_tf(flag.flag)
        if detected == True:
            n+=1

        if n==cycles:
            n+=1
            print("object has been fixed")
            flag.flag = True

        elif n>cycles and flag.flag == False:
            n = 0
        new = datetime.now()
        # print("The time lapse between two target pose publications is: ", new - old)