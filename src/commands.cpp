
#include <iostream>
#include <eigen3/Eigen/Dense>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>

#include <panda_controllers/DesiredProjectTrajectory.h>

#include "ros/ros.h"

#include <sstream>
using namespace std;

Eigen::Vector3d pos_d;
Eigen::Vector3d or_d;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   exit(signum);
}

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
  int inter_x, inter_y, inter_z, comp_x, comp_y, comp_z;

  signal(SIGINT, signal_callback_handler);

  while (ros::ok())
  {
    cout<<"delta_pos: (x y z)"<<endl;
    cin>>dx;
    cin>>dy;
    cin>>dz;
    cout<<"delta_or: (phi (z) thetha (y) psi (x))"<<endl;
    cin>>dpsi;
    cin>>dtheta;
    cin>>dphi;
    cout<<"interaction: "<<endl;
    cin>>inter_x;
    cin>>inter_y;
    cin>>inter_z;
    cout<<"compensation: "<<endl;
    cin>>comp_x;
    cin>>comp_y;
    cin>>comp_z;
    
    ros::spinOnce();

    traj.header.stamp = ros::Time::now();

    traj.pose.position.x = pos_d.x() + dx;
    traj.pose.position.y = pos_d.y() + dy;
    traj.pose.position.z = pos_d.z() + dz;
    traj.pose.orientation.x = or_d.x() + dpsi;
    traj.pose.orientation.y = or_d.y() + dtheta;
    traj.pose.orientation.z = or_d.z() + dphi;
    
    traj.velocity.position.x = 0;
    traj.velocity.position.y = 0;
    traj.velocity.position.z = 0;
    traj.velocity.orientation.x = 0;
    traj.velocity.orientation.y = 0;
    traj.velocity.orientation.z = 0;

    traj.acceleration.position.x = 0;
    traj.acceleration.position.y = 0;
    traj.acceleration.position.z = 0;
    traj.acceleration.orientation.x = 0;
    traj.acceleration.orientation.y = 0;
    traj.acceleration.orientation.z = 0;

    traj.interaction[0] = inter_x;
    traj.interaction[1] = inter_y;
    traj.interaction[2] = inter_z;
    traj.compensation[0] = comp_x;
    traj.compensation[1] = comp_y;
    traj.compensation[2] = comp_z;

    pub_cmd.publish(traj);

    loop_rate.sleep();
  }
  return 0;
}
