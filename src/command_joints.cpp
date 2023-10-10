#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <sensor_msgs/JointState.h>

typedef Eigen::Matrix<double, 7, 1> vec7d;

using std::cout;
using std::cin;
using std::endl;

/* Joints value */
vec7d q_curr;
vec7d q_start;

/* Global flag */
bool init_start = false;
bool end_motion = false;
int flag_traj;

/* Define minjerk parameters */
double minjerk_T;
std::vector<double> minjerk_qf(7);

/* Command duration */
double duration;

/* variables for message */ ;
vec7d position_t, velocity_t, acceleration_t;

/* Obtain joints-state */
void jointsCallback( const sensor_msgs::JointStateConstPtr& msg );

/* Trajectories */
void minjerk    (const double dt_, const vec7d q0);
void stay_in_q0    (const double dt_, const vec7d q0);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_joints_node");
	ros::NodeHandle node_handle;
    double frequency = 1000;
	ros::Rate loop_rate(frequency); // 100 Hz,10 volte più lento del controllore

	/* Publisher */
	ros::Publisher pub_cmd = node_handle.advertise<sensor_msgs::JointState>("command_joints", 1000);

	/* Subscriber */
	ros::Subscriber sub_joints =  node_handle.subscribe<sensor_msgs::JointState>("/joint_states", 1,  &jointsCallback);

	/* Message for /computed_torque_controller/command */
	sensor_msgs::JointState msg_joints;
    
    //minjerk_qf = {0,-0.785398163,0,-2.35619449,0,1.57079632679,0.785398163397};

    if (!node_handle.getParam("minjerk/qf", minjerk_qf) ||
		!node_handle.getParam("minjerk/duration", minjerk_T) ||
        !node_handle.getParam("flag/type", flag_traj)) {
		ROS_ERROR("Could not get trajectory parameters!");
		return false;
	}

    /* initialize trajectory pointer */
    void (*traj_ptr)(const double, const vec7d);    
    switch (flag_traj)
    {
    case 1:
        traj_ptr = minjerk;
        duration = minjerk_T;
        break;
    default:
        traj_ptr = stay_in_q0;
        break;
    }
        
    bool start = false;
	double t_start;
    double period = 1/frequency;
	ros::Time t;
    double dt = 0.0;

    ros::Duration(2.0).sleep();

	while (ros::ok()){
    
		ros::spinOnce();
        t = ros::Time::now();


        if (!start){
            t_start = t.toSec();
            start = true;
        }else if(dt<(duration-period)){
            dt = t.toSec() - t_start;
        }else if(!end_motion){
            cout<<"\n=== tempo terminato ===\n";
            q_start = q_curr;
            traj_ptr = stay_in_q0;
            end_motion = true;
        }
        
        (*traj_ptr)(dt,q_start);
        msg_joints.header.stamp = t;

        /* UPDATE MESSAGE */
		msg_joints.position.resize(7);
        msg_joints.position[0] = position_t(0);
        msg_joints.position[1] = position_t(1);
        msg_joints.position[2] = position_t(2);
        msg_joints.position[3] = position_t(3);
        msg_joints.position[4] = position_t(4);
        msg_joints.position[5] = position_t(5);
        msg_joints.position[6] = position_t(6);

        msg_joints.velocity.resize(7);
        msg_joints.velocity[0] = velocity_t(0);
        msg_joints.velocity[1] = velocity_t(1);
        msg_joints.velocity[2] = velocity_t(2);
        msg_joints.velocity[3] = velocity_t(3);
        msg_joints.velocity[4] = velocity_t(4);
        msg_joints.velocity[5] = velocity_t(5);
        msg_joints.velocity[6] = velocity_t(6);

        msg_joints.effort.resize(7);
        msg_joints.effort[0] = acceleration_t(0);
        msg_joints.effort[1] = acceleration_t(1);
        msg_joints.effort[2] = acceleration_t(2);
        msg_joints.effort[3] = acceleration_t(3);
        msg_joints.effort[4] = acceleration_t(4);
        msg_joints.effort[5] = acceleration_t(5);
        msg_joints.effort[6] = acceleration_t(6);

        pub_cmd.publish(msg_joints);  

		loop_rate.sleep();
		
	}
	return 0;
}

void jointsCallback(const sensor_msgs::JointStateConstPtr& msg){
	vec7d q_tmp(7);
	
	q_tmp(0) = msg->position[0];
	q_tmp(1) = msg->position[1];
	q_tmp(2) = msg->position[2];
	q_tmp(3) = msg->position[3];
	q_tmp(4) = msg->position[4];
	q_tmp(5) = msg->position[5];
	q_tmp(6) = msg->position[6];
	
	q_curr = q_tmp;

	if (!init_start){
		q_start = q_curr;
		init_start = true;
	}
}

void minjerk(const double dt_, const vec7d q0){
	vec7d start, end;

    start = q0;
    end << minjerk_qf[0],minjerk_qf[1],minjerk_qf[2],minjerk_qf[3],minjerk_qf[4],minjerk_qf[5],minjerk_qf[6];

    position_t << start + (start - end)*(15*pow((dt_/duration),4) - 6*pow((dt_/duration),5) -10*pow((dt_/duration),3));
	velocity_t << (start - end)*(60*(pow(dt_,3)/pow(duration,4)) - 30*(pow(dt_,4)/pow(duration,5)) -30*(pow(dt_,2)/pow(duration,3)));
	acceleration_t << (start - end)*(180*(pow(dt_,2)/pow(duration,4)) - 120*(pow(dt_,3)/pow(duration,5)) -60*(dt_/pow(duration,3)));
}

void stay_in_q0(const double dt_, const vec7d q0){

    position_t = q0;
    velocity_t.setZero();
    acceleration_t.setZero();
}