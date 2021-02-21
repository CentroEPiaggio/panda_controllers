
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

#define alpha 0.1

struct traj_struct{
    Eigen::Vector3d pos_des;
    Eigen::Vector3d vel_des;
    Eigen::Vector3d acc_des;
} traj;


Eigen::Vector3d pos;
Eigen::Vector3d orient;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   exit(signum);
}

void poseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  
  pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  orient << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
}

void interpolator(   Eigen::Vector3d pos_i, Eigen::Vector3d pos_f,
                            double tf, double t, 
                            Eigen::Vector3d vel){

    double ta = alpha*tf;                       // time interval of acceleration
    Eigen::Vector3d xa = (1/2)*vel*ta;          // space done during the acceleration
    
    if ((t >= 0) && (t < ta)){
        
        traj.acc_des << vel/ta;
        traj.vel_des << traj.acc_des*t;
        traj.pos_des << pos_i + (1/2)*traj.acc_des*pow(t,2);
    }
    else if ((t >= ta) && (t < (tf-ta))){
        traj.acc_des << 0, 0, 0; 
        traj.vel_des << vel; 
        traj.pos_des << pos_i + xa + traj.pos_des*(t-ta);
    }
    else if ((t >= (tf- ta)) && (t < tf)){
        
        traj.acc_des << -vel/ta;
        traj.vel_des << vel + traj.acc_des*(t - (tf-ta));
        traj.pos_des << pos_i + xa + vel*(tf-2*ta) + vel*(t-(tf-ta)) + (1/2)*traj.acc_des*pow((t-(tf-ta)),2);
    }
    else {
        traj.acc_des << 0, 0, 0;
        traj.vel_des << 0, 0, 0;
        traj.pos_des << pos_f;
    }


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "proj_trajectory");

  ros::NodeHandle node_handle;

  ros::Publisher pub_cmd = node_handle.advertise<panda_controllers::DesiredProjectTrajectory>("/project_impedance_controller/desired_project_trajectory", 1000);

  ros::Subscriber sub_cmd =  node_handle.subscribe("/project_impedance_controller/franka_ee_pose", 1, 
                                                &poseCallback);

  ros::Rate loop_rate(10);

  panda_controllers::DesiredProjectTrajectory traj_msg;
   

  Eigen::Vector3d pos_f;
  double tf;
  Eigen::Vector3d vel;
  Eigen::Vector3d pos_init;


  signal(SIGINT, signal_callback_handler);

  ros::Time t_init;
  double t = 0;

  while (ros::ok()){


    cout<<"time_f "<<endl;
    cin>>tf;
    cout<<"final_position "<<endl;
    cin>> pos_f.x();
    cin>> pos_f.y();
    cin>> pos_f.z();


    // ros::spinOnce();

    t_init = ros::Time::now();
    pos_init = pos;

    while (t <= tf)
    {


      t = (ros::Time::now() - t_init).toSec();

      vel << (pos_f.x() - pos_init.x())/((1-alpha)*tf), (pos_f.y() - pos_init.y())/((1-alpha)*tf), (pos_f.z() - pos_init.z())/((1-alpha)*tf);
      interpolator(pos_init, pos_f, tf, t, vel);

      traj_msg.header.stamp = ros::Time::now();

      traj_msg.pose.position.x = traj.pos_des.x();
      traj_msg.pose.position.y = traj.pos_des.y();
      traj_msg.pose.position.z = traj.pos_des.z();
      traj_msg.pose.orientation.x = orient.x();
      traj_msg.pose.orientation.y = orient.y();
      traj_msg.pose.orientation.z = orient.z();
      
      traj_msg.velocity.position.x = traj.vel_des.x();
      traj_msg.velocity.position.y = traj.vel_des.y();
      traj_msg.velocity.position.z = traj.vel_des.z();
      traj_msg.velocity.orientation.x = 0;
      traj_msg.velocity.orientation.y = 0;
      traj_msg.velocity.orientation.z = 0;

      traj_msg.acceleration.position.x = traj.acc_des.x();
      traj_msg.acceleration.position.y = traj.acc_des.y();
      traj_msg.acceleration.position.z = traj.acc_des.z();
      traj_msg.acceleration.orientation.x = 0;
      traj_msg.acceleration.orientation.y = 0;
      traj_msg.acceleration.orientation.z = 0;

      pub_cmd.publish(traj_msg);

      loop_rate.sleep();
    }
  }
  return 0;
}
