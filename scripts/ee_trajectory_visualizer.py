#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

class TrajectoryVisualizer:
    def __init__(self):
        rospy.init_node('ee_trajectory_visualizer')

        self.listener = tf.TransformListener()

        self.marker_pub = rospy.Publisher('/ee_marker_trajectory', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/ee_path_trajectory', Path, queue_size=10)

        self.trajectory_points = []
        self.path_msg = Path()
        self.path_msg.header.frame_id = "panda_link0"  # o quello base del robot

        self.max_points = 5000  # limita il numero di punti memorizzati

        self.rate = rospy.Rate(20.0)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('panda_link0', 'panda_EE', rospy.Time(0))
                self.add_point(trans, rot)
                self.publish_marker()
                self.publish_path()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            self.rate.sleep()

    def add_point(self, trans, rot):
        p = Point(trans[0], trans[1], trans[2])
        self.trajectory_points.append(p)
        if len(self.trajectory_points) > self.max_points:
            self.trajectory_points.pop(0)

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "panda_link0"
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]

        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.max_points:
            self.path_msg.poses.pop(0)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "panda_link0"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ee_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.005
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = self.trajectory_points

        self.marker_pub.publish(marker)

    def publish_path(self):
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path_msg)


if __name__ == '__main__':
    TrajectoryVisualizer()
