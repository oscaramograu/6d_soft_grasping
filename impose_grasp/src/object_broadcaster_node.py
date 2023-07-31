#!/usr/bin/env python  
from datetime import datetime
import rospy
from std_msgs.msg import String  # Import the ROS message type for the stop broadcasting flag
from impose_grasp.nodes.object_detection.object_broadcaster import ObjectBroadcaster  # Replace "your_package" with the actual package name containing ObjectBroadcaster

class Flag:
    def __init__(self) -> None:
        self.flag = False
        rospy.Subscriber("stop_flag", String, self.callback)

    def callback(self,data):
        print("sdhjknld")
        print(self.flag)

        self.flag = False
        print(self.flag)


if __name__ == "__main__":
    rospy.init_node("object_broadcaster_node")
    flag = Flag()

    object_br = ObjectBroadcaster("cpsduck")

    n=0
    old = datetime.now()
    cycles = 10
    r = rospy.Rate(5) # 10Hz
    while not rospy.is_shutdown():
        object_br.broadcast_tf(flag.flag)

        if n < cycles:
            n+=1
            print("object has been found")

        elif(n==cycles):
            n+=1
            print("object has been fixed")
            flag.flag = True

        else:
            r.sleep()
            if not flag.flag:
                n = 0
                print("Start detecting again")

