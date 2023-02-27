//#include <iostream>
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include <math.h>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>

// #include <panda_controllers/DesiredProjectTrajectory.h>
// #include "utils/parsing_utilities.h"
#include "ros/ros.h"

#include <sstream>
#include <eigen_conversions/eigen_msg.h>

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/JointState.h"
#include "qb_interface/handRef.h"

using namespace std;

bool init = false;
bool  brake_init = false;
double gamma_;
double tf_throw;
double beta;

// TIMES
ros::Time t_init;
double t ; // absolute time
double t_throw, t_brake;
double t_init_throw, t_init_brake;

struct traj_struct
{
  Eigen::Matrix<double, 7, 1> pos_des;
  Eigen::Matrix<double, 7, 1> vel_des;
  Eigen::Matrix<double, 7, 1> acc_des;
} traj;


Eigen::Matrix<double, 7, 1> q_i, q_final, t_finals;

// define braking initial conditions
Eigen::Matrix<double, 7, 1> q_i_brake, dq_i_brake, ddq_brake, ddq_lim;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum)
{
  cout << "Caught signal " << signum << endl;
  // Terminate program
  exit(signum);
}

Eigen::MatrixXd sign(Eigen::MatrixXd v)
{   
    Eigen::Index r = v.rows();
    Eigen::Index c = v.cols();
    Eigen::MatrixXd sign_vect(r,c);
    
    for (int i = 0; i < r; i++)
    {
        for (int j = 0; j < c; j++)
        {   
            if (v(i,j) > 0.0)
            {
                sign_vect(i,j) = +1.0; 
            }
            else if (v(i,j) < 0.0)
            {
                sign_vect(i,j) = -1.0;
            }
            else
            {
                sign_vect(i,j) = 0.0;
            }
        }        
    }
    return sign_vect;
}

// minimum jerk tragectory with dqi, ddqi, ddqf fixed to 0
// expressions obtained from symbolic matlab expressions by using ccode() function in matlab
void minimum_jerk(Eigen::Matrix<double, 7, 1> q_i, Eigen::Matrix<double, 7, 1> q_f, Eigen::Matrix<double, 7, 1> dq_f, double ti, double tf, double t, double beta)
{
  t /= beta;

  traj.pos_des << 1.0/pow(tf-ti,5.0)*(q_f*(t*t*t*t*t)*6.0-q_i*(t*t*t*t*t)*6.0+q_i*(tf*tf*tf*tf*tf)-q_f*(ti*ti*ti*ti*ti)-dq_f*(t * t * t * t * t) * tf * 3.0 - dq_f * t * (ti * ti * ti * ti * ti) + dq_f * (t * t * t * t * t) * ti * 3.0 + dq_f * tf * (ti * ti * ti * ti * ti) - q_f * (t * t * t * t) * tf * 1.5E+1 + q_i * (t * t * t * t) * tf * 1.5E+1 - q_f * (t * t * t * t) * ti * 1.5E+1 + q_i * (t * t * t * t) * ti * 1.5E+1 + q_f * tf * (ti * ti * ti * ti) * 5.0 - q_i * (tf * tf * tf * tf) * ti * 5.0 - dq_f * (t * t * t) * (tf * tf * tf) * 4.0 + dq_f * (t * t * t * t) * (tf * tf) * 7.0 + dq_f * (t * t * t) * (ti * ti * ti) * 6.0 - dq_f * (t * t * t * t) * (ti * ti) * 8.0 - dq_f * (tf * tf) * (ti * ti * ti * ti) * 5.0 + dq_f * (tf * tf * tf) * (ti * ti * ti) * 4.0 + q_f * (t * t * t) * (tf * tf) * 1.0E+1 - q_i * (t * t * t) * (tf * tf) * 1.0E+1 + q_f * (t * t * t) * (ti * ti) * 1.0E+1 - q_i * (t * t * t) * (ti * ti) * 1.0E+1 - q_f * (tf * tf) * (ti * ti * ti) * 1.0E+1 + q_i * (tf * tf * tf) * (ti * ti) * 1.0E+1 + dq_f * (t * t) * (tf * tf) * (ti * ti) * 6.0 + dq_f * t * tf * (ti * ti * ti * ti) * 5.0 + dq_f * (t * t * t * t) * tf * ti + q_f * (t * t * t) * tf * ti * 4.0E+1 - q_i * (t * t * t) * tf * ti * 4.0E+1 + dq_f * t * (tf * tf) * (ti * ti * ti) * 8.0 - dq_f * t * (tf * tf * tf) * (ti * ti) * 1.2E+1 - dq_f * (t * t) * tf * (ti * ti * ti) * 1.8E+1 + dq_f * (t * t) * (tf * tf * tf) * ti * 1.2E+1 + dq_f * (t * t * t) * tf * (ti * ti) * 1.4E+1 - dq_f * (t * t * t) * (tf * tf) * ti * 1.6E+1 + q_f * t * (tf * tf) * (ti * ti) * 3.0E+1 - q_f * (t * t) * tf * (ti * ti) * 3.0E+1 - q_f * (t * t) * (tf * tf) * ti * 3.0E+1 - q_i * t * (tf * tf) * (ti * ti) * 3.0E+1 + q_i * (t * t) * tf * (ti * ti) * 3.0E+1 + q_i * (t * t) * (tf * tf) * ti * 3.0E+1);

  traj.vel_des << -pow(t - ti, 2.0) * 1.0 / pow(tf - ti, 5.0) * (dq_f * (tf * tf * tf) * 1.2E+1 + dq_f * (ti * ti * ti) - q_f * (t * t) * 3.0E+1 + q_i * (t * t) * 3.0E+1 - q_f * (tf * tf) * 3.0E+1 + q_i * (tf * tf) * 3.0E+1 - dq_f * t * (tf * tf) * 2.8E+1 + dq_f * (t * t) * tf * 1.5E+1 + dq_f * t * (ti * ti) * 2.0 - dq_f * (t * t) * ti * 1.5E+1 - dq_f * tf * (ti * ti) * 5.0 - dq_f * (tf * tf) * ti * 8.0 + q_f * t * tf * 6.0E+1 - q_i * t * tf * 6.0E+1 + dq_f * t * tf * ti * 2.6E+1);
  traj.vel_des /= beta;

  traj.acc_des << (t-tf)*(t-ti)*1.0/pow(tf-ti,5.0)*(q_f*t*1.0E+1-q_i*t*1.0E+1-q_f*tf*5.0+q_i*tf*5.0-q_f*ti*5.0+q_i*ti*5.0+dq_f*(tf*tf)*2.0-dq_f*(ti*ti)*3.0-dq_f*t*tf*5.0+dq_f*t*ti*5.0+dq_f*tf*ti)*1.2E+1;
  traj.acc_des /= pow(beta,2);
}

void poseCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  // Throw init
  if (t_throw < tf_throw*beta && init == false)
  {
    q_i = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
    t_init_throw = ros::Time::now().toSec();
    init = true;
    cout << "Throw Initializazion : OK --time = " << t  <<"\n\n";
  }

  // Brake init
  if(t_throw > tf_throw*beta && brake_init == false)
  {
    t_init_brake = ros::Time::now().toSec();

    // q_i_brake = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
    // dq_i_brake = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->velocity).data());
    // Eigen::Matrix<double, 7, 1> dq_i_brake_sign = sign(dq_i_brake); 
    // ddq_brake = -gamma_ * ddq_lim.array() * dq_i_brake_sign.array();

    // // defining final times vector (one for each joint)
    // t_finals = -(dq_i_brake.array() / ddq_brake.array());
    // q_final = q_i_brake.array() + dq_i_brake.array() * t_finals.array() + 0.5 *(ddq_brake.array() * pow(t_finals.array(),2)) ;
    brake_init = true;
    cout << "Brake Initialization: OK --time = " << t  <<"\n\n";
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_pub");
  ros::NodeHandle node_handle;
  ros::Publisher pub_cmd = node_handle.advertise<sensor_msgs::JointState>("/computed_torque_controller/command", 1000);
  ros::Subscriber sub_cmd = node_handle.subscribe<sensor_msgs::JointState>("/franka_state_controller/joint_states", 1, &poseCallback);
  ros::Publisher pub_hand = node_handle.advertise<qb_interface::handRef>("/qb_class/hand_ref",1000);

  // CREATING THE MESSAGES
  sensor_msgs::JointState traj_msg;
  qb_interface::handRef handRefMsg;

  // SET SLEEP TIME TO 1 ms ---> 1 kHz
  ros::Rate loop_rate(1000);
  
  // get tf from ros_parameters (yaml file)
  if (!node_handle.getParam("/computed_torque_controller/tf_throw", tf_throw))
  {
    ROS_ERROR("Trajectory pub: Could not get parameter tf!");
    return false;
  }

  // get gamma from ros_parameters (yaml file)
  if (!node_handle.getParam("/computed_torque_controller/gamma", gamma_))
  {
    ROS_ERROR("Trajectory pub: Could not get parameter gamma!");
    return false;
  }

  // get beta from ros_parameters (yaml file)
  if (!node_handle.getParam("/computed_torque_controller/beta", beta))
  {
    ROS_ERROR("Trajectory pub: Could not get parameter beta!");
    return false;
  }

  // get q_f from ros_parameters (yaml file)
  double q1_f, q2_f, q3_f, q4_f, q5_f, q6_f, q7_f;
  if (!node_handle.getParam("/computed_torque_controller/q1_f", q1_f) || !node_handle.getParam("/computed_torque_controller/q2_f", q2_f) || !node_handle.getParam("/computed_torque_controller/q3_f", q3_f) || !node_handle.getParam("/computed_torque_controller/q4_f", q4_f) || !node_handle.getParam("/computed_torque_controller/q5_f", q5_f) || !node_handle.getParam("/computed_torque_controller/q6_f", q6_f) || !node_handle.getParam("/computed_torque_controller/q7_f", q7_f))
  {
    ROS_ERROR("Trajectory pub: Could not get parameter qf_i (one or more) !");
    return false;
  }

  // get dq_f from ros_parameters (yaml file)
  double dq1_f, dq2_f, dq3_f, dq4_f, dq5_f, dq6_f, dq7_f;
  if (!node_handle.getParam("/computed_torque_controller/dq1_f", dq1_f) || !node_handle.getParam("/computed_torque_controller/dq2_f", dq2_f) || !node_handle.getParam("/computed_torque_controller/dq3_f", dq3_f) || !node_handle.getParam("/computed_torque_controller/dq4_f", dq4_f) || !node_handle.getParam("/computed_torque_controller/dq5_f", dq5_f) || !node_handle.getParam("/computed_torque_controller/dq6_f", dq6_f) || !node_handle.getParam("/computed_torque_controller/dq7_f", dq7_f))
  {
    ROS_ERROR("Trajectory pub: Could not get parameter dqf_i (one or more) !");
    return false;
  }

  // get hand opening time from ros_parameters (yaml file)
  double tOpen;
  if (!node_handle.getParam("/computed_torque_controller/open_time_from_throw", tOpen))
  {
    ROS_ERROR("Trajectory pub: Could not get parameter tOpen!");
    return false;
  }
  
  signal(SIGINT, signal_callback_handler);

  Eigen::Matrix<double, 7, 1> q_f, dq_f;
  q_f << q1_f, q2_f, q3_f, q4_f, q5_f, q6_f, q7_f; 
  dq_f << dq1_f, dq2_f, dq3_f, dq4_f, dq5_f, dq6_f, dq7_f; 
  ddq_lim << 15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0;


  t_init = ros::Time::now();
  t_throw = 0.0;

  bool phase1 = false, phase2 = false;
  
  while (ros::ok())
  {
    // Wait for callback to get q_i
    ros::spinOnce();
    t = (ros::Time::now() - t_init).toSec();
    
    // THROW TRAJECTORY
    while (t_throw <= (tf_throw*beta) && init == true)
    {
      t = (ros::Time::now() - t_init).toSec();  //absolute time 
      

      t_throw = (ros::Time::now().toSec() - t_init_throw);
    
      if(!phase1)
      {
        phase1 = true;
        cout << "THROWING: time = " << t  <<" \n";
      }
      
      minimum_jerk(q_i, q_f, dq_f, 0.0 /*t_i*/, tf_throw, t_throw, beta);

      // Filling message
      traj_msg.header.stamp = ros::Time::now();
      
      std::vector<double> pose_des{traj.pos_des[0], traj.pos_des[1], traj.pos_des[2], traj.pos_des[3], traj.pos_des[4], traj.pos_des[5], traj.pos_des[6]};
      traj_msg.position = pose_des;

      std::vector<double> vel_des{traj.vel_des[0], traj.vel_des[1], traj.vel_des[2], traj.vel_des[3], traj.vel_des[4], traj.vel_des[5], traj.vel_des[6]};
      traj_msg.velocity = vel_des;

      std::vector<double> acc_des{traj.acc_des[0], traj.acc_des[1], traj.acc_des[2], traj.acc_des[3], traj.acc_des[4], traj.acc_des[5], traj.acc_des[6]};
      traj_msg.effort = acc_des;

      // Publishing and sleep
      pub_cmd.publish(traj_msg);

      // Opening hand at tOpen before throwing_time
      if(t_throw >= ((tf_throw*beta) - tOpen)){
        handRefMsg.closure.push_back(30);
        pub_hand.publish(handRefMsg);
      }

      loop_rate.sleep();
    }

    // BRAKING TRAJECTORY
    while (t_throw > tf_throw*beta && brake_init == true)
    {
      t = (ros::Time::now() - t_init).toSec();  //absolute time
      t_brake = (ros::Time::now().toSec() - t_init_brake);

      // evaluating current time
      if(!phase2)
      {
        phase2 = true;
        cout << "BRAKING: time = " << t  <<" \n";
        // bobba
        Eigen::Matrix<double, 7, 1> dq_i_brake_sign = sign(dq_f); 
        ddq_brake = -gamma_ * ddq_lim.array() * dq_i_brake_sign.array();
        // defining final times vector (one for each joint)
        t_finals = -(dq_f.array() / ddq_brake.array());
        cout <<"t_finals = " << t_finals;
        q_final = q_f.array() + dq_f.array() * t_finals.array() + 0.5 *(ddq_brake.array() * pow(t_finals.array(),2)) ;
        dq_i_brake = dq_f;
        q_i_brake = q_f;
        // end bobba
        brake_init = true;
      }

      int n = q_i_brake.rows();
      for (int i = 0; i < n; i++)
      {
        if (t_brake < t_finals(i,1))
        {
          traj.acc_des(i,1) = ddq_brake(i,1);
          traj.vel_des(i,1) = dq_i_brake(i,1) + traj.acc_des(i,1) * t_brake;
          traj.pos_des(i,1) = q_i_brake(i,1) +  dq_i_brake(i,1) *  t_brake + 0.5 * traj.acc_des(i,1) * pow(t_brake,2);
        }
        else
        {
          traj.acc_des(i,1) = 0.0;
          traj.vel_des(i,1) = 0.0;
          traj.pos_des(i,1) = q_final(i,1);
        }
      }

      // Filling message
      traj_msg.header.stamp = ros::Time::now();
      
      std::vector<double> pose_des{traj.pos_des[0], traj.pos_des[1], traj.pos_des[2], traj.pos_des[3], traj.pos_des[4], traj.pos_des[5], traj.pos_des[6]};
      traj_msg.position = pose_des;

      std::vector<double> vel_des{traj.vel_des[0], traj.vel_des[1], traj.vel_des[2], traj.vel_des[3], traj.vel_des[4], traj.vel_des[5], traj.vel_des[6]};
      traj_msg.velocity = vel_des;

      std::vector<double> acc_des{traj.acc_des[0], traj.acc_des[1], traj.acc_des[2], traj.acc_des[3], traj.acc_des[4], traj.acc_des[5], traj.acc_des[6]};
      traj_msg.effort = acc_des;

      // Publishing and sleep
      pub_cmd.publish(traj_msg);
      loop_rate.sleep();
    }

  }
  return 0;
}