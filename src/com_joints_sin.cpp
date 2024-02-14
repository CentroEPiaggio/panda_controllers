#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include "panda_controllers/point.h"
#include "panda_controllers/desTrajEE.h"

#include "utils/ThunderPanda.h"
#include "utils/utils_cartesian.h"

#define NJ 7

typedef Eigen::Vector3d vec3d;

using std::cout;
using std::cin;
using std::endl;

void poseCallback(const panda_controllers::pointConstPtr& msg);
/* End-effector current position */
vec3d pose_EE;
vec3d pose_EE_start;

bool init_start = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "com_joints_sin_node");
	ros::NodeHandle node_handle;
    double frequency = 500;
	ros::Rate loop_rate(frequency);
    double omega_sin_1 = M_PI/2;
    double amp_1 = 0.30;
    double omega_sin_2 = M_PI;
    double amp_2 = 0.60;

    /* Publisher */
	ros::Publisher pub_des_jointState = node_handle.advertise<sensor_msgs::JointState>("command_joints", 1); 

    /* Subscriber */
	ros::Subscriber sub_pose = node_handle.subscribe<panda_controllers::point>("current_config", 1, &poseCallback);

    sensor_msgs::JointState command;
    command.position.resize(NJ);
    command.velocity.resize(NJ);
    command.effort.resize(NJ);

    Eigen::Matrix<double, NJ, 1> qr;
    Eigen::Matrix<double, NJ, 1> dot_qr;
    Eigen::Matrix<double, NJ, 1> ddot_qr;

    ros::Time t;
    double t_start;
    double dt = 0;

    t = ros::Time::now();
    t_start = t.toSec();
    while (ros::ok()){
        
        ros::spinOnce();
        t = ros::Time::now();
        if (dt == 0)
        {
            dt = t.toSec() - t_start;
            cout<<"tempo di inizio:"<<dt;
        }else
        {
            dt = t.toSec() - t_start;
        }
        
        qr << 0.0+amp_1*sin(1.5*(omega_sin_1)*dt), 0.0+amp_1*sin(2*(omega_sin_1/2)*dt), 0.0+amp_1*sin(2*(omega_sin_1/4)*dt), -1.5+amp_1*sin(2*(omega_sin_1/4)*dt), 0.0+amp_2*sin(2*(omega_sin_2/6)*dt), 1.5+amp_2*sin(2*(omega_sin_2/6)*dt), 0.0+amp_2*sin(2*(omega_sin_2/8)*dt);
        dot_qr << 1.5*(omega_sin_1)*amp_1*cos(1.5*(omega_sin_1)*dt), 2*(omega_sin_1/2)*amp_1*cos(2*(omega_sin_1/2)*dt), 2*(omega_sin_1/4)*amp_1*cos(2*(omega_sin_1/4)*dt), 2*(omega_sin_1/4)*amp_1*cos(2*(omega_sin_1/4)*dt), 2*(omega_sin_2/6)*amp_2*cos(2*(omega_sin_2/6)*dt), 2*(omega_sin_2/6)*amp_2*cos(2*(omega_sin_2/6)*dt), 2*(omega_sin_2/8)*amp_2*cos(2*(omega_sin_2/8)*dt);
        ddot_qr << -pow(1.5*(omega_sin_1),2)*amp_1*sin(1.5*(omega_sin_1)*dt), -pow(2*(omega_sin_1/2),2)*amp_1*sin(2*(omega_sin_1/2)*dt), +pow(2*(omega_sin_1/4),2)*amp_1*sin(2*(omega_sin_1/4)*dt), -pow(2*(omega_sin_1/4),2)*amp_1*sin(2*(omega_sin_1/4)*dt), -pow(2*(omega_sin_2/6),2)*amp_2*sin(2*(omega_sin_2/6)*dt), -pow(2*(omega_sin_2/6),2)*amp_2*sin(2*(omega_sin_2/6)*dt), -pow(2*(omega_sin_2/8),2)*amp_2*sin(2*(omega_sin_2/8)*dt);
        
        for(int i=0;i<NJ;i++){
            command.position[i] = qr(i);
            command.velocity[i] = dot_qr(i);
            command.effort[i] = ddot_qr(i);
        }

        pub_des_jointState.publish(command);  
		loop_rate.sleep();

    }
    return 0;
}

void poseCallback(const panda_controllers::pointConstPtr& msg){

	double EE_x, EE_y, EE_z;
	
	EE_x = msg->xyz.x;
	EE_y = msg->xyz.y;
	EE_z = msg->xyz.z;

	pose_EE << EE_x, EE_y, EE_z;

	if (!init_start){
		pose_EE_start = pose_EE;
		init_start = true;
        std::cout<<"\n==============\n"<<"pose_EE_start:"<<"\n==============\n"<<pose_EE_start<<"\n==============\n";
	}
}