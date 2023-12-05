#include <iostream>
#include <eigen3/Eigen/Dense>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>

#include "ros/ros.h"

#include <sstream>
// #include <eigen_conversions/eigen_msg.h>

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
// #include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"

using namespace std;

#define alpha 0.1
bool init_flag = false;
bool init_q0 = false;

struct traj_struct{
	Eigen::Matrix<double, 7, 1> pos_des;
	Eigen::Matrix<double, 7, 1> vel_des;
	Eigen::Matrix<double, 7, 1> acc_des;
} traj;

// define q0 as 7x1 matrix
Eigen::Matrix<double, 7, 1> q0;
const double q_lim_upp[] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
const double q_lim_low[] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum){
	cout << "Caught signal " << signum << endl;
	// Terminate program
	exit(signum);
}

// void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
//   pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//   orient << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
// }

void jointsCallback( const sensor_msgs::JointStateConstPtr& msg ){
	q0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
	init_q0 = true;
}

void interpolator_pos(Eigen::Matrix<double, 7, 1> pos_i, Eigen::Matrix<double, 7, 1> pos_f, double tf, double t){
	traj.pos_des << pos_i + (pos_i - pos_f)*(15*pow((t/tf),4) - 6*pow((t/tf),5) -10*pow((t/tf),3));
	traj.vel_des << (pos_i - pos_f)*(60*(pow(t,3)/pow(tf,4)) - 30*(pow(t,4)/pow(tf,5)) -30*(pow(t,2)/pow(tf,3)));
	traj.acc_des << (pos_i - pos_f)*(180*(pow(t,2)/pow(tf,4)) - 120*(pow(t,3)/pow(tf,5)) -60*(t/pow(tf,3)));
}


// void demo_inf_XY(Eigen::Vector3d pos_i, double t){
// 	Eigen::Vector3d tmp;
// 	tmp << sin(t)/8, sin(t/2)/4, 0;
// 	traj.pos_des << pos_i + tmp;
// 	traj.vel_des << cos(t)/8, cos(t/2)/8, 0;
// 	traj.acc_des << -sin(t)/8, -sin(t/2)/16, 0;
// }

// void demo_inf_XYZ(Eigen::Vector3d pos_i, double t,double zf,double tf){
// 	Eigen::Vector3d tmp;
// 	tmp << sin(t)/8, sin(t/2)/4, ((zf-pos_i(2))/tf)*t;
// 	traj.pos_des << pos_i + tmp;
// 	traj.vel_des << cos(t)/8, cos(t/2)/8, (zf-pos_i(2))/tf;
// 	traj.acc_des << -sin(t)/8, -sin(t/2)/16, 0;
// }

// void demo_circle_xy(Eigen::Vector3d pos_i, double t,double zf,double tf){
//   Eigen::Vector3d tmp;
//   tmp << 0.1*cos(t), 0.1*sin(t), ((zf-pos_i(2))/tf)*t;
//   traj.pos_des << pos_i + tmp;
//   traj.vel_des << -0.1*sin(t), 0.1*cos(t), (zf-pos_i(2))/tf;
//   traj.acc_des << -0.1*cos(t), -0.1*sin(t), 0;
// }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "menu");

	ros::NodeHandle node_handle;

	ros::Publisher pub_cmd = node_handle.advertise<sensor_msgs::JointState>("/computed_torque_controller/command", 1000);
	ros::Subscriber sub_joints =  node_handle.subscribe<sensor_msgs::JointState>("/franka_state_controller/joint_states", 1, &jointsCallback);
	// ros::Subscriber sub_pose =  node_handle.subscribe("/franka_state_controller/franka_ee_pose", 1, &poseCallback);

	// creating trajectory message
	sensor_msgs::JointState traj_msg;

	// SET SLEEP TIME 1000 ---> 1 kHz
	ros::Rate loop_rate(200);

	srand(time(NULL));
	double tf;
	Eigen::Matrix<double, 7, 1> qf;
	XmlRpc::XmlRpcValue menu_par;

	// Initialize Ctrl-C
	signal(SIGINT, signal_callback_handler);

	ros::Time t_init;
	init_q0 = false;
	double t = 0;
	int choice;
	int demo = -1;
	int yaml = 0;

	while (ros::ok()){
		demo = -1;
		if (yaml==1){
			choice = 5;
		}else{
			cout<<"choice:   (1: joints min-jerk,  2: go to init,  3: go to random  4: yaml) "<<endl;
			cin>>choice;
		}
		if (choice == 1){
			cout<<"duration: "<<endl;
			cin>>tf;
			cout<<"final_joint_positions: "<<endl;
			cin>> qf(0);
			cin>> qf(1);
			cin>> qf(2);
			cin>> qf(3);
			cin>> qf(4);
			cin>> qf(5);
			cin>> qf(6);
		}else if (choice == 2){
			std::vector<double> qf_array;
			if(!node_handle.getParam("/menu/Q0_INIT", qf_array))
				ROS_ERROR("Failed to get parameter from server.");
			qf = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(qf_array.data(), qf_array.size());
			choice = 1;
			tf = 3.0;
		}else if (choice == 3){
			for(int i=0; i<7; i++){
				double q_low = q_lim_low[i];
				double q_upp = q_lim_upp[i];
				qf(i) = q_low + (float(rand())/RAND_MAX)*(q_upp - q_low);
			}
			choice = 1;
			tf = 3.0;
		}else if (choice == 4){
			cout<<"-not implemented yet-"<<endl;
		}

		if (!init_flag){
			init_q0 = false;
			init_flag = true;
		}

		ros::spinOnce();

		t_init = ros::Time::now();

		t = (ros::Time::now() - t_init).toSec();

		while (t <= tf && init_q0)
		{
			if (choice == 1){
				interpolator_pos(q0, qf, tf, t);
			} else if (choice == 4){
				break;
			} else {
				break;
			}

			traj_msg.header.stamp = ros::Time::now();

			std::vector<double> pos_des {traj.pos_des[0], traj.pos_des[1],traj.pos_des[2],traj.pos_des[3],traj.pos_des[4],traj.pos_des[5],traj.pos_des[6]};
			traj_msg.position = pos_des;
			std::vector<double> vel_des {traj.vel_des[0], traj.vel_des[1],traj.vel_des[2],traj.vel_des[3],traj.vel_des[4],traj.vel_des[5],traj.vel_des[6]};
			traj_msg.velocity = vel_des;
			std::vector<double> acc_des {traj.acc_des[0], traj.acc_des[1],traj.acc_des[2],traj.acc_des[3],traj.acc_des[4],traj.acc_des[5],traj.acc_des[6]};
			traj_msg.effort = acc_des;

			pub_cmd.publish(traj_msg);

			loop_rate.sleep();

			t = (ros::Time::now() - t_init).toSec();
		}
	}
	return 0;
}