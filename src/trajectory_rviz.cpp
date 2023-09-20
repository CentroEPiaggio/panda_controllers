#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "panda_controllers/point.h"
#include <eigen3/Eigen/Core>

Eigen::Vector3d pose_EE;

void configCallback(const panda_controllers::point& msg) {
    
    double EE_x, EE_y, EE_z;
	
	EE_x = msg.xyz.x;
	EE_y = msg.xyz.y;
	EE_z = msg.xyz.z;

	pose_EE << EE_x, EE_y, EE_z;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "trajectory_visualizer");
    ros::NodeHandle node_handle;
    ros::Rate rate(20);

    ros::Publisher vanishing_pub = node_handle.advertise<visualization_msgs::Marker>("vanishing_marker", 1);
    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("trajectory_marker", 1);
    
    ros::Subscriber config_sub = node_handle.subscribe("/backstepping_controller/current_config", 1, &configCallback);   
    //ros::Subscriber config_sub = node_handle.subscribe("/computed_torque_controller/current_config", 1, &configCallback);
    
    visualization_msgs::Marker total_traj;
    total_traj.header.frame_id = "panda_link0";
    total_traj.ns = "trajectory_markers";
    total_traj.id = 1;
    total_traj.type = visualization_msgs::Marker::POINTS;
    total_traj.action = visualization_msgs::Marker::ADD;
    total_traj.pose.orientation.w = 1.0;
    total_traj.scale.x = 0.005;
    total_traj.scale.y = 0.005;
    total_traj.scale.z = 0.005;
    total_traj.color.r = 1.0;
    total_traj.color.g = 1.0;
    total_traj.color.b = 0.0;
    total_traj.color.a = 0.9;

    visualization_msgs::Marker vanishing;
    vanishing.header.frame_id = "panda_link0";
    vanishing.ns = "trajectory_markers";
    vanishing.id = 0;
    vanishing.type = visualization_msgs::Marker::LINE_STRIP;
    vanishing.action = visualization_msgs::Marker::ADD;
    vanishing.pose.orientation.w = 1.0;
    vanishing.scale.x = 0.02; // Spessore delle linee
    vanishing.scale.y = 0.02;
    vanishing.scale.z = 0.02;
    vanishing.color.r = 1.0;
    vanishing.color.a = 1.0;
   
    while (ros::ok()) {

        geometry_msgs::Point p;
        p.x = pose_EE(0);
        p.y = pose_EE(1);
        p.z = pose_EE(2);

        vanishing.points.push_back(p);
        total_traj.points.push_back(p);
        
        ros::Time time_now = ros::Time::now();
        total_traj.header.stamp = time_now;
        if (vanishing.points.size()>50){vanishing.points.erase(vanishing.points.begin());}
        vanishing.header.stamp = time_now;
        
        marker_pub.publish(total_traj);
        vanishing_pub.publish(vanishing);
                
        ros::spinOnce();
        rate.sleep();
    
    }    
    return 0;
}
