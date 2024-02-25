#include <eigen3/Eigen/Dense>
#include <signal.h>
#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <iostream>

using namespace std;

ros::Publisher pub_hand_qbh1;
ros::Publisher pub_hand_qbh2;
int close_hand = 1;
float effort;
int GRIPPER = 0;

// ----- Functions ----- //
void qbhand1_move(float synergy);
void qbhand2_move(float synergy, float manipulation);

// ----- Callbacks ----- //
// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum){
	cout << "Caught signal " << signum << endl;
	// Terminate program
	exit(signum);
}

void callback_close(const std_msgs::Int32& msg){
	close_hand = msg.data;
	// ROS_INFO("Message hand-close arrived!");
}

void callback_effort1(const sensor_msgs::JointState& msg){
	effort = msg.effort[0];
	// ROS_INFO("Message effort arrived!");
}
void callback_effort2(const sensor_msgs::JointState& msg){
	effort = (msg.effort[1] + msg.effort[2])/2;
	// ROS_INFO("Message effort arrived!");
}

// ----- MAIN ----- //
int main (int argc, char **argv)
{
	ros::init(argc, argv, "close_hand_node");
	ros::NodeHandle nh;

	// Node parameters
	int RATE;
	float RATE_CLOSE;
	float HAND_EFFORT_MAX;
	if(!nh.getParam("/throw_node/RATE", RATE))
		ROS_ERROR("Failed to get parameter from server.");
	if(!nh.getParam("/throw_node/GRIPPER", GRIPPER))
		ROS_ERROR("Failed to get parameter from server.");
	if(!nh.getParam("/throw_node/RATE_CLOSE", RATE_CLOSE))
		ROS_ERROR("Failed to get parameter from server.");
	if(!nh.getParam("/throw_node/HAND_EFFORT_MAX", HAND_EFFORT_MAX))
		ROS_ERROR("Failed to get parameter from server.");

	ros::Rate rate(RATE);
	ros::Rate rate_close(RATE_CLOSE);

	// Pub-Sub
	ros::Subscriber sub_hand1 = nh.subscribe("/robot/gripper/qbhand1/control/joint_states", 1, &callback_effort1);
	ros::Subscriber sub_hand2 = nh.subscribe("/robot/gripper/qbhand2m1/control/joint_states", 1, &callback_effort2);
	ros::Subscriber sub_close = nh.subscribe("/robot/gripper/effort_close", 1, &callback_close);
	pub_hand_qbh1 = nh.advertise<trajectory_msgs::JointTrajectory>("/robot/gripper/qbhand1/control/qbhand1_synergy_trajectory_controller/command", 1);
    pub_hand_qbh2 = nh.advertise<trajectory_msgs::JointTrajectory>("/robot/gripper/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command", 1);

	float closure = 0;
	effort = 0;

	while(ros::ok()){
		if (close_hand){
			// closing hand
			float i = 0.1;
			closure = 0;
			while (effort<HAND_EFFORT_MAX){
				closure  = i;
				if (GRIPPER == 1)
					qbhand1_move(closure);
				else if (GRIPPER == 2)
					qbhand2_move(closure,0);
				rate_close.sleep();
				ros::spinOnce();
				i += 0.05;
				if (i >= 1.03) break;
				cout<<"closure: "<<closure<<endl;
				cout<<"effort: "<<effort<<endl;
			}
			close_hand = 0;
		}
		rate.sleep();
		ros::spinOnce();
	}
}

void qbhand2_move(float synergy, float manipulation){
	trajectory_msgs::JointTrajectory joint_traj_msg;

    // Header
    joint_traj_msg.header.stamp = ros::Time::now();
    joint_traj_msg.header.seq = 0;
    joint_traj_msg.header.stamp.sec = 0;
    joint_traj_msg.header.stamp.nsec = 0;

    // Define Joint Names
    joint_traj_msg.joint_names = {"qbhand2m1_manipulation_joint", "qbhand2m1_synergy_joint"};

    // Positions, Velocities, Accelerations, Effort and time_from_start
    trajectory_msgs::JointTrajectoryPoint joint_traj_point_msg;

    joint_traj_point_msg.positions = {manipulation, synergy};
    joint_traj_point_msg.velocities = {0, 0};
    joint_traj_point_msg.accelerations = {0, 0};
    joint_traj_point_msg.effort = {0, 0};
    joint_traj_point_msg.time_from_start.sec = 1;

    // Assign
    joint_traj_msg.points.push_back(joint_traj_point_msg);

    // Publish the message to close the hand
    pub_hand_qbh2.publish(joint_traj_msg);

    ROS_INFO("Hand moved!");
}

/*Function for closing and opening the softhand1 */
void qbhand1_move(float synergy){

    trajectory_msgs::JointTrajectory joint_traj_msg;

    // Header
    joint_traj_msg.header.stamp = ros::Time::now();
    joint_traj_msg.header.seq = 0;
    joint_traj_msg.header.stamp.sec = 0;
    joint_traj_msg.header.stamp.nsec = 0;

    // Define Joint Names
    joint_traj_msg.joint_names = {"qbhand1_synergy_joint"};

    // Positions, Velocities, Accelerations, Effort and time_from_start
    trajectory_msgs::JointTrajectoryPoint joint_traj_point_msg;

    joint_traj_point_msg.positions = {synergy};
    joint_traj_point_msg.velocities = {0};
    joint_traj_point_msg.accelerations = {0};
    joint_traj_point_msg.effort = {0};
    joint_traj_point_msg.time_from_start.sec = 1;

    // Assign
    joint_traj_msg.points.push_back(joint_traj_point_msg);

    // Publish the message to close the hand
    pub_hand_qbh1.publish(joint_traj_msg);
    ROS_INFO("Softhand1 moved!");
}
