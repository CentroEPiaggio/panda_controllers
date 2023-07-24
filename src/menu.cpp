#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <geometry_msgs/Point.h>
#include "panda_controllers/desTrajEE.h"

using std::cout;
using std::cin;
using std::endl;

/* End-effector current position */
Eigen::Vector3d pose_EE;
Eigen::Vector3d pose_EE_start;

/* Trajectory desired in time t */
Eigen::VectorXd trajectory_t(9);

/* Global flag */
bool exit_flag = false;
bool init_start = false;

/* Lissajous trajectory and its derivate */
void lissajous(double t_sym, double A, double B, double a, double b, double phi) {
    
    Eigen::Vector3d pos_des;
    Eigen::Vector3d vel_des;
    Eigen::Vector3d acc_des;
	
    double xstart = 0.5;
    double ystart = 0;
    double zstart = 0.4;

    pos_des << 0 + xstart, A * std::cos(a * t_sym + phi) + ystart, B * std::sin(b * t_sym) + zstart; 	// position
	vel_des << 0, - A * a * std::sin(a * t_sym + phi), B * b * std::cos(b * t_sym);						// velocity
	acc_des << 0, - A * a * a * std::cos(a * t_sym + phi), -B * b * b * std::sin(b * t_sym);			// acceleration
	trajectory_t << pos_des, vel_des, acc_des;
}

/* Interpolate trajectory with NOT min jerk in CARTESIAN */
void interpolator_pos(Eigen::Vector3d pos_i, Eigen::Vector3d pos_f, double tf, double t){
	
	Eigen::Vector3d pos_des;
    Eigen::Vector3d vel_des;
    Eigen::Vector3d acc_des;
	
	pos_des << pos_i + (pos_i - pos_f)*(15*pow((t/tf),4) - 6*pow((t/tf),5) -10*pow((t/tf),3));
	vel_des << (pos_i - pos_f)*(60*(pow(t,3)/pow(tf,4)) - 30*(pow(t,4)/pow(tf,5)) -30*(pow(t,2)/pow(tf,3)));
	acc_des << (pos_i - pos_f)*(180*(pow(t,2)/pow(tf,4)) - 120*(pow(t,3)/pow(tf,5)) -60*(t/pow(tf,3)));
	trajectory_t << pos_des, vel_des, acc_des;
}

/* Fill message for cartesian command */
void fillMsgCartesian(panda_controllers::desTrajEE& msg) {
    
	msg.position.x = trajectory_t(0);
    msg.position.y = trajectory_t(1);
    msg.position.z = trajectory_t(2);

    msg.velocity.x = trajectory_t(3);
    msg.velocity.y = trajectory_t(4);
    msg.velocity.z = trajectory_t(5);

    msg.acceleration.x = trajectory_t(6);
    msg.acceleration.y = trajectory_t(7);
    msg.acceleration.z = trajectory_t(8);
}

/* Define the function to be called when ctrl-c (SIGINT) is sent to process */
void signal_callback_handler(int signum){
	
	cout << "Caught signal " << signum << endl;
	// Terminate program
	exit(signum);
}

/* Obatain end-effector pose */
void poseCallback(const geometry_msgs::Point& msg){

	double EE_x, EE_y, EE_z;
	
	EE_x = msg.x;
	EE_y = msg.y;
	EE_z = msg.z;

	pose_EE << EE_x, EE_y, EE_z;

	if (!init_start){
		pose_EE_start = pose_EE;
		init_start = true;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "menu");
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(200);
	ros::Time t_init;
	
	/* Publisher */
	ros::Publisher pub_cmd_cartesian = node_handle.advertise<panda_controllers::desTrajEE>("/backstepping_controller/command", 1000);
	//ros::Publisher pub_new_cmd = node_handle.advertise<>("/backstepping_controller/new_command_flag", 10);
	
	/* Subscriber */
	ros::Subscriber sub_pose = node_handle.subscribe("/backstepping_controller/current_config", 1, &poseCallback);

	/* Message */
	panda_controllers::desTrajEE msg_cartesian;
	
	/* Initialize Ctrl-C */
	signal(SIGINT, signal_callback_handler);

	/* Define pose of end-effector desired by user */
	Eigen::Vector3d pose_des_user;
	
	double t;
	double tf;
	double delta_tf;

	int choice;
	bool start_ = true;
	
	while (ros::ok() && !exit_flag){
		
		/* Lissajous */
		double A1, A2, f1, f2, phi;
		if (!node_handle.getParam("Amp1", A1) || !node_handle.getParam("Amp2", A2) ||
			!node_handle.getParam("freq1", f1) || !node_handle.getParam("freq2", f2) ||
			!node_handle.getParam("phi", phi)) {
			ROS_ERROR("Could not get parameters A1, A2, f1, f2, phi!");
			return false;
		}
		
		start_ = true;

		if (ros::Time::now().toSec()>1){

			delta_tf = 0.0;
	
			cout<<"\n\nChoice:   (1: Lissajous, 2: Position Control, 3: Return to start position, 0:Exit) "<<endl;
			cin>>choice;
			
			if (choice == 1){
				cout<<" duration: "<<endl;
				cin>>delta_tf;
			}
			else if (choice == 2){
				cout<<" duration: "<<endl;
				cin>>delta_tf;
				cout<<" desired position: "<<endl;
				cout<<" position x: ";
				cin>>pose_des_user(0);
				cout<<" position y: ";
				cin>>pose_des_user(1);
				cout<<" position z: ";
				cin>>pose_des_user(2);
			}
			else if (choice == 3){
				cout<<" duration: "<<endl;
				cin>>delta_tf;
			}
			else if (choice == 0){
				cout<<"\n Exiting... "<<endl;
				cout<<" Run roslaunch panda_controllers menu.launch to restart menu node \n "<<endl;
				exit_flag = true;
			}
			else{
				cout<<"Error input"<<endl;
				start_ = false;
			}

			ros::spinOnce();

			t_init = ros::Time::now();
			t = t_init.toSec();
			tf = t + delta_tf;

			while (t <= tf && start_)
			{
				
				if (choice == 1){
					lissajous(t-t_init.toSec(), A1, A2, f1, f2, phi);					
				} else if (choice == 2){
					interpolator_pos(pose_EE, pose_des_user, tf, t);
				} else if (choice == 3){
					interpolator_pos(pose_EE, pose_EE_start, tf, t);
				} else if (choice == 0){
					trajectory_t << pose_EE, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
				}

				fillMsgCartesian(msg_cartesian);
				msg_cartesian.header.stamp = ros::Time::now();
				pub_cmd_cartesian.publish(msg_cartesian);

				loop_rate.sleep();
				
				t = ros::Time::now().toSec();
			}

		}
	}

	return 0;
}