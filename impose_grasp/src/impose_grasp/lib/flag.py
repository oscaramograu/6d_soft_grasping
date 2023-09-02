import rospy
from std_msgs.msg import String

class Flag:
    def __init__(self) -> None:
        self.flag = False
        rospy.Subscriber("restart_flag", String, self.callback)

    def __call__(self):
        return self.flag
    
    def set_flag(self, value):
        self.flag = value

    def callback(self, msg):
        print("Restarting detection")
        self.flag = True
