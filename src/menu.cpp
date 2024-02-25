#include <iostream>
#include <eigen3/Eigen/Dense>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>
#include "panda_controllers/desTrajEE.h"

#include "ros/ros.h"

#include <sstream>
// #include <eigen_conversions/eigen_msg.h>

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
// #include "std_msgs/Int32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
// #include "geometry_msgs/Pose.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// #include <dh_gripper_msgs/GripperCtrl.h>
// #include <dh_gripper_msgs/GripperRotCtrl.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include "sensor_msgs/JointState.h"
#include "franka_msgs/FrankaState.h"
#include "franka_msgs/ErrorRecoveryActionGoal.h"
// #include "panda_controllers/CommandParams.h"
// #include "panda_controllers/Commands.h"

// // #define ROBOT_NAME "/robot/arm"	// real robot
// #define ROBOT_NAME ""	// simulation
#define alpha 0.1

using namespace std;

// ----- Variables ----- //
int switch_ack = 0;
bool init_flag = false;
bool init_q0 = false;
Eigen::Matrix<double, 7, 1> q0;
Eigen::Matrix<double, 3, 1> p0;
Eigen::Matrix<double, 3, 1> p0_saved;
Eigen::Matrix<double, 3, 1> pos;
Eigen::Matrix<double, 4, 1> orient;
const double q_lim_upp[] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
const double q_lim_low[] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
ros::Publisher pub_hand_qbh1;
ros::Publisher pub_hand_qbh2;
// ros::Publisher pub_dh3_rot;
// ros::Publisher pub_dh3_ctrl;

struct traj_struct_joints{
	Eigen::Matrix<double, 7, 1> pos_des;
	Eigen::Matrix<double, 7, 1> vel_des;
	Eigen::Matrix<double, 7, 1> acc_des;
} traj_joints;

struct traj_struct_cartesian{
	Eigen::Matrix<double, 3, 1> pos_des;
	Eigen::Matrix<double, 3, 1> vel_des;
	Eigen::Matrix<double, 3, 1> acc_des;
} traj_cartesian;

// ----- Functions ----- //
void qbhand1_move(float synergy);
void qbhand2_move(float synergy, float manipulation);
void dh3_rot(float rot);
void dh3_ctrl(float ctrl);

// ----- Callbacks ----- //
void callback_switch(const std_msgs::Int32& msg){
	switch_ack = msg.data;
}

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum){
	cout << "Caught signal " << signum << endl;
	// Terminate program
	exit(signum);
}

// void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
// 	pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
// 	orient << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
// }

void frankaCallback(const franka_msgs::FrankaStateConstPtr& msg){
	Eigen::Matrix3d rotation;
	rotation << msg->O_T_EE[0], msg->O_T_EE[4], msg->O_T_EE[8],
				msg->O_T_EE[1], msg->O_T_EE[5], msg->O_T_EE[9],
				msg->O_T_EE[2], msg->O_T_EE[6], msg->O_T_EE[10];
	Eigen::Quaterniond quaternion(rotation);
	quaternion.normalize();
	pos[0] = msg->O_T_EE[12];
	pos[1] = msg->O_T_EE[13];
	pos[2] = msg->O_T_EE[14];
	Eigen::Vector4d coeffs = quaternion.coeffs();
	orient(0) = coeffs(0);
	orient(1) = coeffs(1);
	orient(2) = coeffs(2);
	orient(3) = coeffs(3);
	q0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->q).data());
	p0 << pos[0], pos[1], pos[2];
	if (init_q0 == false) p0_saved = p0;
	init_q0 = true;
}

void jointsCallback( const sensor_msgs::JointStateConstPtr& msg ){
	q0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
	//init_q0 = true;
}

void interpolator_joints(Eigen::Matrix<double, 7, 1> pos_i, Eigen::Matrix<double, 7, 1> pos_f, double tf, double t){
	traj_joints.pos_des << pos_i + (pos_i - pos_f)*(15*pow((t/tf),4) - 6*pow((t/tf),5) -10*pow((t/tf),3));
	traj_joints.vel_des << (pos_i - pos_f)*(60*(pow(t,3)/pow(tf,4)) - 30*(pow(t,4)/pow(tf,5)) -30*(pow(t,2)/pow(tf,3)));
	traj_joints.acc_des << (pos_i - pos_f)*(180*(pow(t,2)/pow(tf,4)) - 120*(pow(t,3)/pow(tf,5)) -60*(t/pow(tf,3)));
}

void interpolator_cartesian(Eigen::Matrix<double, 3, 1> pos_i, Eigen::Matrix<double, 3, 1> pos_f, double tf, double t){
	traj_cartesian.pos_des << pos_i + (pos_i - pos_f)*(15*pow((t/tf),4) - 6*pow((t/tf),5) -10*pow((t/tf),3));
	traj_cartesian.vel_des << (pos_i - pos_f)*(60*(pow(t,3)/pow(tf,4)) - 30*(pow(t,4)/pow(tf,5)) -30*(pow(t,2)/pow(tf,3)));
	traj_cartesian.acc_des << (pos_i - pos_f)*(180*(pow(t,2)/pow(tf,4)) - 120*(pow(t,3)/pow(tf,5)) -60*(t/pow(tf,3)));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "menu");
	ros::NodeHandle node_handle;
	std::string robot_name;// = ROBOT_NAME;
	int SIMULATION = 1;
	int GRIPPER = 2;
	int CONTROLLER = 2;	// ( 1: CT_joints,  2: CT_cartesian,  3: Backstepping,  4: joint_trajectory )"

	if(!node_handle.getParam("/menu/SIMULATION", SIMULATION))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/menu/GRIPPER", GRIPPER))
		ROS_ERROR("Failed to get parameter from server.");

	if (SIMULATION){
		robot_name = "";
	}else{
		robot_name = "";
		// robot_name = "/robot/arm";
	}

	ros::Publisher pub_switch = node_handle.advertise<std_msgs::Int32>(robot_name + "/menu/switch_controller", 1);
	ros::Publisher pub_traj_joints = node_handle.advertise<sensor_msgs::JointState>(robot_name + "/computed_torque_controller/command_joints", 10);
	ros::Publisher pub_traj_cartesian = node_handle.advertise<panda_controllers::desTrajEE>(robot_name + "/CT_mod_controller_OS/command_cartesian", 10);
	ros::Publisher pub_error = node_handle.advertise<franka_msgs::ErrorRecoveryActionGoal>(robot_name + "/franka_control/error_recovery/goal", 1);
	// ros::Subscriber sub_joints = node_handle.subscribe<sensor_msgs::JointState>(robot_name + "/franka_state_controller/joint_states", 1, &jointsCallback);
	ros::Subscriber sub_franka = node_handle.subscribe<franka_msgs::FrankaState>(robot_name + "/franka_state_controller/franka_states", 1, &frankaCallback);
	pub_hand_qbh1 = node_handle.advertise<trajectory_msgs::JointTrajectory>("/robot/gripper/qbhand1/control/qbhand1_synergy_trajectory_controller/command", 1);
    pub_hand_qbh2 = node_handle.advertise<trajectory_msgs::JointTrajectory>("/robot/gripper/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command", 1);
	// pub_dh3_ctrl = node_handle.advertise<dh_gripper_msgs::GripperCtrl>("/robot/gripper/ctrl", 1);
	// // pub_dh3_rot = node_handle.advertise<dh_gripper_msgs::GripperRotCtrl>("/robot/gripper/ctrl", 1);
	// ros::Subscriber sub_switch_ack = node_handle.subscribe(robot_name + "/menu/switch_ack", 1, &callback_switch);
	
	// creating trajectory message
	// panda_controllers::Commands command_msg;
	sensor_msgs::JointState command_joints_msg;
	panda_controllers::desTrajEE command_cartesian_msg;

	// SET SLEEP TIME 1000 ---> 1 kHz
	ros::Rate loop_rate(200);

	srand(time(NULL));
	double tf = 3.0;
	Eigen::Matrix<double, 7, 1> qf;
	Eigen::Matrix<double, 3, 1> pf;
	XmlRpc::XmlRpcValue menu_par;

	// Initialize Ctrl-C
	signal(SIGINT, signal_callback_handler);

	ros::Time t_init;
	init_q0 = false;
	float q0_saved[7];
	for(int i=0; i<7; i++){
		double q_low = q_lim_low[i];
		double q_upp = q_lim_upp[i];
		q0_saved[i] = (q_low + q_upp) / 2;
	}
	double t = 0;
	int choice = 0;
	int choice2 = 0;
	int demo = -1;
	int yaml = 0;

	while (ros::ok()){
		demo = -1;
		if (yaml==1){
			choice = 4;
		}else{
			cout<<"choice:  (1: go to,  2: save configuration,  3: error recover,  4: gripper control,  5: switch controller,  6: yaml (not implemented) )"<<endl;
			cin>>choice;
		}
		if (choice == 1){
			// - go to - //
			cout<<"go to:  ( 1: init,  2: navigation,  3: input,  4: saved,  5: random,     9: set duration,  0: cancel ) "<<endl;
			cin >> choice2;
			if (choice2 == 1){
				// - go to init - //
				if (CONTROLLER == 1){
					std::vector<double> qf_array;
					if(!node_handle.getParam("/menu/Q0_INIT", qf_array))
						ROS_ERROR("Failed to get parameter from server.");
					qf = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(qf_array.data(), qf_array.size());
				}
			}else if (choice2 == 2){
				// - go to navigation - //
				if (CONTROLLER == 1){
					std::vector<double> Q0_NAV;
					if(!node_handle.getParam("/menu/Q0_NAV", Q0_NAV))
						ROS_ERROR("Failed to get parameter from server.");
					qf = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(Q0_NAV.data(), Q0_NAV.size());
				}
			}else if (choice2 == 3){
				// - go to input - //
				if (CONTROLLER == 1){
					cout<<"final_joint_positions: "<<endl;
					cin>> qf(0);
					cin>> qf(1);
					cin>> qf(2);
					cin>> qf(3);
					cin>> qf(4);
					cin>> qf(5);
					cin>> qf(6);
				}else if (CONTROLLER == 2){
					cout<<"final_EE_position: "<<endl;
					cin>> pf(0);
					cin>> pf(1);
					cin>> pf(2);
				}
			}else if (choice2 == 4){
				// - go to saved - //
				if (CONTROLLER == 1){
					for(int i=0; i<7; i++){
						qf(i) = q0_saved[i];
					}
				}else if (CONTROLLER == 2){
					for(int i=0; i<3; i++){
						pf(i) = p0_saved[i];
					}
				}
			}else if (choice2 == 5){
				// - go to random - //
				if (CONTROLLER == 1){
					if (SIMULATION){
						for(int i=0; i<7; i++){
							double q_low = q_lim_low[i];
							double q_upp = q_lim_upp[i];
							qf(i) = q_low + (float(rand())/RAND_MAX)*(q_upp - q_low);
						}
					}
				}
			}else if (choice2 == 9){
				// - set duration - //
				cout<<"duration: "<<endl;
				cin>>tf;
			}else if (choice2 == 9){
			}else {
				cout<<"not valid!"<<endl;
			}
		}else if (choice == 2){
			// - save configuration - //
			init_q0 = false;
			while(init_q0==false){
				ros::spinOnce();
				loop_rate.sleep();
			}
			for(int i=0; i<7; i++){
				q0_saved[i] = q0[i];
			}
			for(int i=0; i<3; i++){
				p0_saved[i] = p0[i];
			}
			cout<<"q: ["<<q0[0]<<", "<<q0[1]<<", "<<q0[2]<<", "<<q0[3]<<", "<<q0[4]<<", "<<q0[5]<<", "<<q0[6]<<"]"<<endl;
			cout<<"ee_pose: ["<<pos[0]<<", "<<pos[1]<<", "<<pos[2]<<", "<<orient[0]<<", "<<orient[1]<<", "<<orient[2]<<", "<<orient[3]<<"]"<<endl;
			cout<<"saved!"<<endl;
		}else if (choice == 3){
			// - error recovering - //
			franka_msgs::ErrorRecoveryActionGoal error_msg;
			pub_error.publish(error_msg);
		}else if (choice == 4){
			// - Gripper Control - //
			float value;
			cout<<"value: "<<endl;
			cin>>value;
			if (GRIPPER == 0){
				// dh3_ctrl(value);
			}else if (GRIPPER == 1){
				qbhand1_move(value);
			}else if (GRIPPER == 2){
				qbhand2_move(value,0);
			}
		}else if (choice == 5){
			// - Swich controller - //
			cout << "select controller:  ( 1: CT_joints,  2: CT_cartesian,  3: Backstepping,  4: joint_trajectory )"<<endl;
			cin >> choice2;
			switch_ack = 0;
			std_msgs::Int32 switch_msg;
			switch_msg.data = choice2;
			pub_switch.publish(switch_msg);
			while (!switch_ack){
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
		ros::spinOnce();

		t_init = ros::Time::now();

		t = (ros::Time::now() - t_init).toSec();

		while (t <= tf && init_q0)
		{
			if (choice == 1){
				if (CONTROLLER == 1){
					interpolator_joints(q0, qf, tf, t);
				}else if (CONTROLLER == 2){
					interpolator_cartesian(p0, pf, tf, t);
				}
					
			} else if (choice == 4){
				break;
			} else {
				break;
			}

			if (CONTROLLER == 1){
				command_joints_msg.header.stamp = ros::Time::now();
				// std::vector<double> pos_des {traj_joints.pos_des[0], traj_joints.pos_des[1],traj_joints.pos_des[2],traj_joints.pos_des[3],traj_joints.pos_des[4],traj_joints.pos_des[5],traj_joints.pos_des[6]};
				// command_joints_msg.joint_commands.position = pos_des;
				// std::vector<double> vel_des {traj_joints.vel_des[0], traj_joints.vel_des[1],traj_joints.vel_des[2],traj_joints.vel_des[3],traj_joints.vel_des[4],traj_joints.vel_des[5],traj_joints.vel_des[6]};
				// command_joints_msg.joint_commands.velocity = vel_des;
				// std::vector<double> acc_des {traj_joints.acc_des[0], traj_joints.acc_des[1],traj_joints.acc_des[2],traj_joints.acc_des[3],traj_joints.acc_des[4],traj_joints.acc_des[5],traj_joints.acc_des[6]};
				// command_joints_msg.joint_commands.effort = acc_des;
				command_joints_msg.position.resize(7);
				command_joints_msg.position[0] = traj_joints.pos_des(0);
				command_joints_msg.position[1] = traj_joints.pos_des(1);
				command_joints_msg.position[2] = traj_joints.pos_des(2);
				command_joints_msg.position[3] = traj_joints.pos_des(3);
				command_joints_msg.position[4] = traj_joints.pos_des(4);
				command_joints_msg.position[5] = traj_joints.pos_des(5);
				command_joints_msg.position[6] = traj_joints.pos_des(6);
				command_joints_msg.velocity.resize(7);
				command_joints_msg.velocity[0] = traj_joints.vel_des(0);
				command_joints_msg.velocity[1] = traj_joints.vel_des(1);
				command_joints_msg.velocity[2] = traj_joints.vel_des(2);
				command_joints_msg.velocity[3] = traj_joints.vel_des(3);
				command_joints_msg.velocity[4] = traj_joints.vel_des(4);
				command_joints_msg.velocity[5] = traj_joints.vel_des(5);
				command_joints_msg.velocity[6] = traj_joints.vel_des(6);
				command_joints_msg.effort.resize(7);
				command_joints_msg.effort[0] = traj_joints.acc_des(0);
				command_joints_msg.effort[1] = traj_joints.acc_des(1);
				command_joints_msg.effort[2] = traj_joints.acc_des(2);
				command_joints_msg.effort[3] = traj_joints.acc_des(3);
				command_joints_msg.effort[4] = traj_joints.acc_des(4);
				command_joints_msg.effort[5] = traj_joints.acc_des(5);
				command_joints_msg.effort[6] = traj_joints.acc_des(6);
				pub_traj_joints.publish(command_joints_msg);
			}else if (CONTROLLER == 2){
				command_cartesian_msg.position.x = traj_cartesian.pos_des[0];
				command_cartesian_msg.position.y = traj_cartesian.pos_des[1];
				command_cartesian_msg.position.z = traj_cartesian.pos_des[2];
				command_cartesian_msg.velocity.x = traj_cartesian.vel_des[0];
				command_cartesian_msg.velocity.y = traj_cartesian.vel_des[1];
				command_cartesian_msg.velocity.z = traj_cartesian.vel_des[2];
				command_cartesian_msg.acceleration.x = traj_cartesian.acc_des[0];
				command_cartesian_msg.acceleration.y = traj_cartesian.acc_des[1];
				command_cartesian_msg.acceleration.z = traj_cartesian.acc_des[2];
				pub_traj_cartesian.publish(command_cartesian_msg);
			}

			loop_rate.sleep();

			t = (ros::Time::now() - t_init).toSec();
		}
		loop_rate.sleep();
	}
	return 0;
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

    joint_traj_point_msg.positions = {synergy, manipulation};
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

// void dh3_rot(float rot){
//     // Fingers rotation
//     dh_gripper_msgs::GripperRotCtrl dh3_rot_msg;
//     dh3_rot_msg.angle = rot;
//     pub_dh3_rot.publish(dh3_rot_msg);
// }

// void dh3_ctrl(float ctrl){
//     // Fingers closure
//     dh_gripper_msgs::GripperCtrl dh3_ctrl_msg;
//     dh3_ctrl_msg.position = ctrl;
//     pub_dh3_ctrl.publish(dh3_ctrl_msg);
// }