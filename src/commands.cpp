
#include <iostream>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>

#include <panda_controllers/DesiredProjectTrajectory.h>

#include "ros/ros.h"

#include <sstream>
using namespace std;

Eigen::Vector3d pos_d;
Eigen::Vector3d or_d;

void poseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  
  pos_d << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  or_d << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "commands");

  ros::NodeHandle node_handle;

  ros::Publisher pub_cmd = node_handle.advertise<panda_controllers::DesiredProjectTrajectory>("/project_impedance_controller/desired_project_trajectory", 1000);

  ros::Subscriber sub_cmd =  node_handle.subscribe("/project_impedance_controller/franka_ee_pose", 1, 
                                                &poseCallback);

  ros::Rate loop_rate(10);

  panda_controllers::DesiredProjectTrajectory traj;
   
  double dx, dy, dz, dphi, dpsi, dtheta;
  while (ros::ok())
  {
    cout<<"delta_pos: enter each coeff"<<endl;
    cin>>dx;
    cin>>dy;
    cin>>dz;
    cout<<"delta_or: enter each coeff"<<endl;
    cin>>dpsi;
    cin>>dtheta;
    cin>>dphi;
    
    ros::spinOnce();

    traj.pose.position.x = pos_d.x() + dx;
    traj.pose.position.y = pos_d.y() + dy;
    traj.pose.position.z = pos_d.z() + dz;
    traj.pose.orientation.x = or_d.x() + dpsi;
    traj.pose.orientation.y = or_d.y() + dtheta;
    traj.pose.orientation.z = or_d.z() + dphi;
    
    pub_cmd.publish(traj);

    loop_rate.sleep();
  }
  return 0;
}
