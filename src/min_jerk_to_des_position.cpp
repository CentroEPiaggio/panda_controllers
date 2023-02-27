#include <iostream>
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>

//#include <panda_controllers/DesiredProjectTrajectory.h>
//#include "utils/parsing_utilities.h"
#include "ros/ros.h"

#include <sstream>
#include <eigen_conversions/eigen_msg.h>

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/JointState.h"

using namespace std;


bool init = true;
double t = 0;

struct traj_struct{
    Eigen::Matrix<double, 7, 1> pos_des;
    Eigen::Matrix<double, 7, 1> vel_des;
    Eigen::Matrix<double, 7, 1> acc_des;
} traj;


// define q_0 as 7x1 matrix
Eigen::Matrix<double, 7, 1> q_0;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   exit(signum);
}

void poseCallback( const sensor_msgs::JointStateConstPtr& msg ) 
{
  //cout <<"ok_callback" <<endl;
  if (init == true)
  {
    q_0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
    init = false;
    //cout << "ok3" <<endl;
  }
}


void interpolator_pos(Eigen::Matrix<double, 7, 1> pos_i, Eigen::Matrix<double, 7, 1> pos_f, double tf, double t)
{

      traj.pos_des << pos_i + (pos_i - pos_f)*(15*pow((t/tf),4) - 6*pow((t/tf),5) -10*pow((t/tf),3));
      traj.vel_des << (pos_i - pos_f)*(60*(pow(t,3)/pow(tf,4)) - 30*(pow(t,4)/pow(tf,5)) -30*(pow(t,2)/pow(tf,3)));
      traj.acc_des << (pos_i - pos_f)*(180*(pow(t,2)/pow(tf,4)) - 120*(pow(t,3)/pow(tf,5)) -60*(t/pow(tf,3)));

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_pub");

  ros::NodeHandle node_handle;

  ros::Publisher pub_cmd = node_handle.advertise<sensor_msgs::JointState>("/computed_torque_controller/command", 1000);
  ros::Subscriber sub_cmd =  node_handle.subscribe<sensor_msgs::JointState>("/franka_state_controller/joint_states", 1,  &poseCallback);

  // CREATING THE MESSAGE
  sensor_msgs::JointState traj_msg;

  // SET SLEEP TIME TO 1000 ms ---> 1 kHz
  ros::Rate loop_rate(1000);

  ros::Time t_init = ros::Time::now();

  // get tf from ros_parameters (yaml file)
  double tf; 
  if (!node_handle.getParam("/computed_torque_controller/tf_des_pos", tf)) 
  {
      ROS_ERROR("Trajectory pub: Could not get parameter tf!");
      return false;
  }
  
  // get q_i from ros_parameters (yaml file)
  double q1_i, q2_i, q3_i, q4_i, q5_i, q6_i, q7_i;
  if (!node_handle.getParam("/computed_torque_controller/q1_i", q1_i) || !node_handle.getParam("/computed_torque_controller/q2_i", q2_i) || !node_handle.getParam("/computed_torque_controller/q3_i", q3_i) || !node_handle.getParam("/computed_torque_controller/q4_i", q4_i) || !node_handle.getParam("/computed_torque_controller/q5_i", q5_i) || !node_handle.getParam("/computed_torque_controller/q6_i", q6_i) || !node_handle.getParam("/computed_torque_controller/q7_i", q7_i)) 
  {
        ROS_ERROR("Trajectory pub: Could not get parameter qf_i (one or more) !");
        return false;
  }

  signal(SIGINT, signal_callback_handler);

  Eigen::Matrix<double, 7, 1> q_i;
  // cout << "ok1" <<endl;
  q_i(0,0) = q1_i; 
  q_i(1,0) = q2_i; 
  q_i(2,0) = q3_i; 
  q_i(3,0) = q4_i;
  q_i(4,0) = q5_i;
  q_i(5,0) = q6_i;
  q_i(6,0) = q7_i;
// cout << "ok2" <<endl;
bool flag = 1;

  while (ros::ok()&& flag==1)
  {
   
    ros::spinOnce();

    t = (ros::Time::now() - t_init).toSec();
    while (t <= tf && init == false)
    {
      flag=0;
      interpolator_pos(q_0, q_i, tf, t);

      traj_msg.header.stamp = ros::Time::now();
      //cout << "ok4" <<endl;
      std::vector<double> pose_des {traj.pos_des[0], traj.pos_des[1],traj.pos_des[2],traj.pos_des[3],traj.pos_des[4],traj.pos_des[5],traj.pos_des[6]};
            
      traj_msg.position = pose_des;

      //cout << "ok5" <<endl;
      std::vector<double> vel_des {traj.vel_des[0], traj.vel_des[1],traj.vel_des[2],traj.vel_des[3],traj.vel_des[4],traj.vel_des[5],traj.vel_des[6]};
      
      traj_msg.velocity = vel_des;


      std::vector<double> acc_des {traj.acc_des[0], traj.acc_des[1],traj.acc_des[2],traj.acc_des[3],traj.acc_des[4],traj.acc_des[5],traj.acc_des[6]};
      
      traj_msg.effort = acc_des;


      pub_cmd.publish(traj_msg);
      // cout <<t <<endl;
      loop_rate.sleep();
      t = (ros::Time::now() - t_init).toSec();
    }

  }
  return 0;
}