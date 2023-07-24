#include <iostream>
#include <eigen3/Eigen/Dense>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>
// #include <panda_controllers/DesiredProjectTrajectory.h>
// #include <panda_controllers/cubeRef.h>

#include "ros/ros.h"

#include <sstream>

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"

// #include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"

using namespace std;

#define alpha 0.1
bool init = true;

struct traj_struct{
	Eigen::Matrix<double, 7, 1> pos_des;
	Eigen::Matrix<double, 7, 1> vel_des;
	Eigen::Matrix<double, 7, 1> acc_des;
} traj;


// define q0 as 7x1 matrix
Eigen::Matrix<double, 7, 1> q0;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum){
	cout << "Caught signal " << signum << endl;
	// Terminate program
	exit(signum);
}

void jointsCallback( const sensor_msgs::JointStateConstPtr& msg ){
	//cout <<"ok_callback" <<endl;
	if (init == true){
		q0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
		init = false;
		//cout << "ok3" <<endl;
	}
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

	// ros::Publisher pub_cube = node_handle.advertise<panda_controllers::cubeRef>("/qb_class/cube_ref",1);

	ros::Publisher pub_cmd = node_handle.advertise<sensor_msgs::JointState>("/computed_torque_controller/command", 1000);
	ros::Subscriber sub_joints =  node_handle.subscribe<sensor_msgs::JointState>("/franka_state_controller/joint_states", 1,  &jointsCallback);
	// ros::Subscriber sub_pose =  node_handle.subscribe("/franka_state_controller/franka_ee_pose", 1, &poseCallback);

	// CREATING THE MESSAGE
	sensor_msgs::JointState traj_msg;
	// panda_controllers::DesiredTrajectory traj_msg;
	// panda_controllers::cubeRef cube_msg;

	// SET SLEEP TIME TO 1000 ms ---> 1 kHz
	ros::Rate loop_rate(200);

	// Eigen::Vector3d pos_f;
	// Eigen::Vector3d or_f;
	double tf;
	Eigen::Matrix<double, 7, 1> qf;
	// double zf;
	// Eigen::Vector3d vel;
	// Eigen::Vector3d pos_init;
	// Eigen::Vector3d or_init;
	// XmlRpc::XmlRpcValue traj_par;
	// std::map<std::string, std::vector<double>>  traj_par; 

	// int N_ACTION;
	// Eigen::MatrixXd ACTIONS;
	// Eigen::MatrixXi TYPE;

	// Initialize Ctrl-C
	signal(SIGINT, signal_callback_handler);

	ros::Time t_init;
	double t = 0;
	int choice;
	int demo = -1;
	int yaml = 0;
	// std::vector<float> cube_theta1 = {0.0};
	// std::vector<float> cube_theta2 = {0.0};
	// float cube_eq;

	// float CUBE_RS;
	while (ros::ok()){
		demo = -1;
		if (yaml==1){
			choice = 5;
		}else{
			cout<<"choice:   (1: joints min-jerk, 2:demos, 3:yaml) "<<endl;
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
			cout<<"-not implemented yet-"<<endl;
			// cin>>demo;
			// cout<<"insert time_f: "<<endl;
			// cin>>tf;
			// if ((demo==2) || (demo==3)){
			// 	cout<<"insert zf: "<<endl;
			// 	cin>>zf;
			// }
		}else if (choice == 3){
			cout<<"-not implemented yet-"<<endl;
			// if (yaml==0){
			// 	yaml = 1;
			// 	if(!node_handle.getParam("/traj_par", traj_par)){
			// 	ROS_ERROR("Could not get the XmlRpc value.");
			// 	}
			// 	if(!parseParameter(traj_par, N_ACTION, "N_ACTION")){
			// 	ROS_ERROR("Could not parse traj_par.");
			// 	}
			// 	if(!parseParameter(traj_par, ACTIONS, "ACTIONS")){
			// 	ROS_ERROR("Could not parse traj_par.");
			// 	}
			// 	if(!parseParameter(traj_par, TYPE, "TYPE")){
			// 	ROS_ERROR("Could not parse traj_par."); 
			// 	}
			// 	// if(!parseParameter(traj_par, CUBE_RS, "CUBE_RS")){
			// 	//   ROS_ERROR("Could not parse traj_par."); 
			// 	// }
			// 	if(!node_handle.getParam("/traj_par/CUBE_RS", CUBE_RS)){
			// 	ROS_ERROR("Could not get the XmlRpc value.");
			// 	}
			// }
			// if(n_act == N_ACTION){
			// 	yaml = 0;
			// 	n_act = 0;
			// }else{
			// 	tf = ACTIONS(n_act,0);
			// 	pos_f(0) = ACTIONS(n_act,1);
			// 	pos_f(1) = ACTIONS(n_act,2);
			// 	pos_f(2) = ACTIONS(n_act,3);
			// 	inter_x = TYPE(n_act,0);
			// 	inter_y = TYPE(n_act,1);
			// 	inter_z = TYPE(n_act,2);
			// 	comp_x = TYPE(n_act,3);
			// 	comp_y = TYPE(n_act,4);
			// 	comp_z = TYPE(n_act,5);
			// 	cube_eq = ACTIONS(n_act,4);
			// 	cube_theta1[0] = CUBE_RS + cube_eq;
			// 	cube_theta2[0] = cube_eq - CUBE_RS;
			// 	choice = 1;
			// 	n_act++;
			// }
		}

		ros::spinOnce();

		// // pos_init = pos;
		// if(yaml==0){
		// 	pos_init = pos;
		// }else{
		// 	if (n_act==1){
		// 		pos_init = pos;
		// 	}else{
		// 		pos_init(0) = ACTIONS(n_act-2,1);
		// 		pos_init(1) = ACTIONS(n_act-2,2);
		// 		pos_init(2) = ACTIONS(n_act-2,3);
		// 	}
		// }

		t_init = ros::Time::now();

		t = (ros::Time::now() - t_init).toSec();

		while (t <= tf && init == false)
		{
			if (choice == 1){
				// interpolator_pos(pos_init, pos_f, tf, t, vel);
				interpolator_pos(q0, qf, tf, t);
			} else if (choice == 2){
				break;
				// if (demo == 1){
				// demo_inf_XY(pos_init, t);
				// }else if(demo==2){
				// demo_inf_XYZ(pos_init,t,zf,tf);
				// }else if (demo==3){
				// cube_theta1[0] = 2400;
				// cube_theta2[0] = -1600;
				// demo_circle_xy(pos_init,t,zf,tf);
				// }
			}

			traj_msg.header.stamp = ros::Time::now();

			std::vector<double> pose_des {traj.pos_des[0], traj.pos_des[1],traj.pos_des[2],traj.pos_des[3],traj.pos_des[4],traj.pos_des[5],traj.pos_des[6]};
			traj_msg.position = pose_des;
			std::vector<double> vel_des {traj.vel_des[0], traj.vel_des[1],traj.vel_des[2],traj.vel_des[3],traj.vel_des[4],traj.vel_des[5],traj.vel_des[6]};
			traj_msg.velocity = vel_des;
			std::vector<double> acc_des {traj.acc_des[0], traj.acc_des[1],traj.acc_des[2],traj.acc_des[3],traj.acc_des[4],traj.acc_des[5],traj.acc_des[6]};
			traj_msg.effort = acc_des;

			// traj_msg.pose.position.x = traj.pos_des.x();
			// traj_msg.pose.position.y = traj.pos_des.y();
			// traj_msg.pose.position.z = traj.pos_des.z();
			// traj_msg.pose.orientation.x = orient.x();
			// traj_msg.pose.orientation.y = orient.y();
			// traj_msg.pose.orientation.z = orient.z();
			
			// traj_msg.velocity.position.x = traj.vel_des.x();
			// traj_msg.velocity.position.y = traj.vel_des.y();
			// traj_msg.velocity.position.z = traj.vel_des.z();
			// traj_msg.velocity.orientation.x = 0;
			// traj_msg.velocity.orientation.y = 0;
			// traj_msg.velocity.orientation.z = 0;

			// traj_msg.acceleration.position.x = traj.acc_des.x();
			// traj_msg.acceleration.position.y = traj.acc_des.y();
			// traj_msg.acceleration.position.z = traj.acc_des.z();
			// traj_msg.acceleration.orientation.x = 0;
			// traj_msg.acceleration.orientation.y = 0;
			// traj_msg.acceleration.orientation.z = 0;

			pub_cmd.publish(traj_msg);

			// cube_msg.p_1 = cube_theta1;
			// cube_msg.p_2 = cube_theta2;
			// pub_cube.publish(cube_msg);

			loop_rate.sleep();

			t = (ros::Time::now() - t_init).toSec();
		}
	}
	return 0;
}