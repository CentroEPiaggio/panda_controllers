#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//Mathematical
#include <iostream>
#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <std_msgs/String.h>

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/types.h>

//Franka
#include <franka/robot_state.h>
#include <franka_msgs/FrankaState.h>

//ROS message
#include <sensor_msgs/JointState.h>

namespace panda_controllers
{

class sub_pub
{

private:

    /*Defining useful items*/

//ROS Time
    double t_final;
//ROS Rate
    double frequency;
    std::shared_ptr<ros::Rate> rate;

//Variables
    double dt; //parameter that is given from the topic
    double t_i,t_f; //t_f is given from the topic
    double dt_hat,t_star;
    double toll; //tollerance

//Subscriber
    ros::Subscriber sub_q_actual, sub_q_desired;
//Publisher
    ros::Publisher pub_q_desired;

//Matrices
    Eigen::Matrix<double,7,1> q_0;
    Eigen::Matrix<double,7,1> q_final;
    Eigen::Matrix<double,7,1> q_d_sym;

//NodeHandle
    ros::NodeHandle node_handle;

//Flag for the new message
    bool flag;

//Array
    std::array<double,7> q_d_star;

//Message
    sensor_msgs::JointState states;


//Inizialization of Publisher and Subscriber functions
    void initializeSub_actual();
    void initializeSub_desired();
    void initializePub();

//Sampling the trajectory
    void sampling_trajectory ( double dt, double t_f );

//Callback's Functions
    void Position_Callback ( const franka_msgs::FrankaStatePtr& msg );

    void Desired_Callback ( const sensor_msgs::JointStatePtr& msg );

public:

    sub_pub ( ros::NodeHandle* node_handle );

};

}

#endif
