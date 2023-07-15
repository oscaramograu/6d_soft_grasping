#!/usr/bin/env python  
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.init_node('point_plotter', anonymous=True)
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

# Create a marker message
marker = Marker()
marker.header.frame_id = "base_link"
marker.type = Marker.POINTS
marker.action = Marker.ADD
marker.scale.x = 0.05  # Point size
marker.scale.y = 0.05
marker.color.a = 1.0  # Alpha (transparency)
marker.color.r = 1.0  # Red
marker.color.g = 0.0  # Green
marker.color.b = 0.0  # Blue

# Set the point position
point = Point()
point.x = 0.5  # X coordinate
point.y = 0.5  # Y coordinate
point.z = 0.5  # Z coordinate
marker.points.append(point)

# Publish the marker
marker_pub.publish(marker)

# Keep the program alive to maintain the marker in RViz
rospy.spin()
