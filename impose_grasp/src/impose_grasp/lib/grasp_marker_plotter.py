import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from geometry_msgs.msg import Point
from typing import List

class MarkerPlotter:
    def __init__(self) -> None:
        self.marker_arr_pub = rospy.Publisher(
            'visualization_marker_array',
             MarkerArray, queue_size=100)
        self.marker_arr = MarkerArray()
        self.id = 0
        self._rate = rospy.Rate(10)  # 10 Hz


    def add_marker(self, pose: np.ndarray, valid: bool):
        """
        Creates a blue or red marker depending if the marker is valid or not.
        Needs to be shown by rviz using show_marker().
        """
        if valid:
            red = 0
            blue = 1
        else:
            red = 1
            blue = 0
        # Create a marker message
        marker = Marker()
        marker.header.frame_id = "panda_link0"
        marker.id = self.id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.005 # Point size
        marker.scale.y = 0.005
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = red # Red
        marker.color.g = 0.0  # Green
        marker.color.b = blue  # Blue

        point = Point(pose[0,3], pose[1,3], pose[2,3])
        marker.points.append(point)

        self.marker_arr.markers.append(marker)
        self.id+=1


    def visualize_array(self):
        print("Markers can now be visualised")
        print("The number of markers is: ", len(self.marker_arr.markers))
        while not rospy.is_shutdown():
            self.marker_arr_pub.publish(self.marker_arr)
            self._rate.sleep()