#!/usr/bin/env python  
import rospy
from visualization_msgs.msg import Marker

def wall_publisher():
    rospy.init_node('wall_marker_publisher')
    pub = rospy.Publisher('/wall_marker', Marker, queue_size=10)

    marker = Marker()
    marker.header.frame_id = "panda_link0_sc"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "obstacle"
    marker.id = 1
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = 0.11  # Cambia con le tue coordinate
    marker.pose.position.y = -0.50
    marker.pose.position.z = 0.35
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.02 
    marker.scale.y = 0.4
    marker.scale.z = 0.7
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.8

    rate = rospy.Rate(1)  # Pubblica ogni secondo
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    wall_publisher()
