#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.msg import ModelStates

class BallMarkerManager:
    def __init__(self):
        rospy.init_node('ball_marker_publisher')
        
        # Publisher per MarkerArray
        self.pub = rospy.Publisher('/ball_markers_array', MarkerArray, queue_size=10)
        
        # Subscriber a Gazebo
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        
        # Dizionario per mappare i nomi dei modelli Gazebo agli ID dei Marker
        self.target_models = {"palla": 1, "palla2": 2}

    def create_marker(self, name, model_id, pose):
        marker = Marker()
        marker.header.frame_id = "world" 
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacle"
        marker.id = model_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose = pose
        
        # Dimensioni della sfera (diametro 10cm)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Colore verde
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        return marker

    def gazebo_callback(self, msg):
        # --- CORREZIONE INDENTAZIONE ---
        if rospy.is_shutdown():
            return

        new_marker_array = MarkerArray()
        
        for i, name in enumerate(msg.name):
            if name in self.target_models:
                m = self.create_marker(name, self.target_models[name], msg.pose[i])
                new_marker_array.markers.append(m)
        
        try:
            if new_marker_array.markers and not rospy.is_shutdown():
                self.pub.publish(new_marker_array)
        except (rospy.ROSException, rospy.ROSInterruptException):
            pass

if __name__ == '__main__':
    try:
        manager = BallMarkerManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass