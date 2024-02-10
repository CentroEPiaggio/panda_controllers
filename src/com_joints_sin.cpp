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
        
        qr << 0.0027+0.30*sin((M_PI)*dt), -0.30+0.30*sin((M_PI/2)*dt), 0.2778+0.30*sin((M_PI/4)*dt), -2.200+0.30*sin((M_PI/4)*dt), 0.2949+0.30*sin((M_PI/6)*dt), 1.5708+0.30*sin((M_PI/6)*dt), 0.7854+0.30*sin((M_PI/8)*dt);
        dot_qr << (M_PI)*0.30*cos((M_PI)*dt), (M_PI/2)*0.30*cos((M_PI/2)*dt), (M_PI/4)*0.30*cos((M_PI/4)*dt), (M_PI/4)*0.30*cos((M_PI/4)*dt), (M_PI/6)*0.30*cos((M_PI/6)*dt), (M_PI/6)*0.30*cos((M_PI/6)*dt), (M_PI/8)*0.30*cos((M_PI/8)*dt);
        ddot_qr << -pow((M_PI),2)*0.30*sin((M_PI)*dt), -pow((M_PI/2),2)*0.30*sin((M_PI/2)*dt), +pow((M_PI/4),2)*0.30*sin((M_PI/4)*dt), -pow((M_PI/4),2)*0.30*sin((M_PI/4)*dt), -pow((M_PI/6),2)*0.30*sin((M_PI/6)*dt), -pow((M_PI/6),2)*0.30*sin((M_PI/6)*dt), -pow((M_PI/8),2)*0.30*sin((M_PI/8)*dt);
        
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