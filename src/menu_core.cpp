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
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "franka_msgs/FrankaState.h"
// #include "panda_controllers/Commands.h"
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

// // #define ROBOT_NAME "/robot/arm"	// real robot
// #define ROBOT_NAME ""	// simulation

using namespace std;

int switch_flag = 0;
bool init_q0 = false;
Eigen::Matrix<double, 7, 1> q0;
std::vector<double> q0_init;
std::vector<double> q0_nav;
// creating trajectory message
// panda_controllers::Commands command_msg;
ros::Publisher pub_traj;

struct traj_struct{
	Eigen::Matrix<double, 7, 1> pos_des;
	Eigen::Matrix<double, 7, 1> vel_des;
	Eigen::Matrix<double, 7, 1> acc_des;
} traj;

// A controller_mangager msg for switching controllers
controller_manager_msgs::SwitchController switch_controller;
// The switch controller service name
std::string switch_service_name = "/controller_manager/switch_controller";
std::string list_controllers_name = "/controller_manager/list_controllers";

// ----- Functions ----- //
// To switch the controllers
bool switch_controllers(std::string robot_name, std::string to_controller);

void min_jerk_joints(Eigen::Matrix<double, 7, 1> pos_i, Eigen::Matrix<double, 7, 1> pos_f, double tf, double t){
	traj.pos_des << pos_i + (pos_i - pos_f)*(15*pow((t/tf),4) - 6*pow((t/tf),5) -10*pow((t/tf),3));
	traj.vel_des << (pos_i - pos_f)*(60*(pow(t,3)/pow(tf,4)) - 30*(pow(t,4)/pow(tf,5)) -30*(pow(t,2)/pow(tf,3)));
	traj.acc_des << (pos_i - pos_f)*(180*(pow(t,2)/pow(tf,4)) - 120*(pow(t,3)/pow(tf,5)) -60*(t/pow(tf,3)));
}

void min_jerk_cartesian(Eigen::Matrix<double, 3, 1> pos_i, Eigen::Matrix<double, 3, 1> pos_f, double tf, double t){
	traj.pos_des << pos_i + (pos_i - pos_f)*(15*pow((t/tf),4) - 6*pow((t/tf),5) -10*pow((t/tf),3));
	traj.vel_des << (pos_i - pos_f)*(60*(pow(t,3)/pow(tf,4)) - 30*(pow(t,4)/pow(tf,5)) -30*(pow(t,2)/pow(tf,3)));
	traj.acc_des << (pos_i - pos_f)*(180*(pow(t,2)/pow(tf,4)) - 120*(pow(t,3)/pow(tf,5)) -60*(t/pow(tf,3)));
}

// ----- Callbacks ----- //
void callback_switch(const std_msgs::Int32& msg){
	if (switch_flag == 0){
		switch_flag = msg.data;
		ROS_INFO("switch controller request arrived!");
	}
}

void callback_goto(const std_msgs::Int32& msg){
	ros::Rate rate(200);
	int choice2 = msg.data;
	double tf = 3.0;
	ros::Time t_init;
	double t = 0.0;
	Eigen::Matrix<double, 7, 1> qf;
	init_q0 = false;
	while (!init_q0){
		ros::spinOnce();
		rate.sleep();
	}
	if (choice2 == 1){
		// - go to init - //
		qf = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(q0_init.data(), q0_init.size());
	}else if (choice2 == 2){
		// - go to navigation - //
		qf = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(q0_nav.data(), q0_nav.size());
	}else if (choice2 == 3){
	}else if (choice2 == 4){
	}else if (choice2 == 5){
	}else{
		choice2 = 0;
	}

	if ((choice2==1)||(choice2==2)){
		t_init = ros::Time::now();
		t = (ros::Time::now() - t_init).toSec();
		while (t <= tf && init_q0)
		{
			min_jerk_joints(q0, qf, tf, t);
			// command_msg.header.stamp = ros::Time::now();
			// std::vector<double> pos_des {traj.pos_des[0], traj.pos_des[1],traj.pos_des[2],traj.pos_des[3],traj.pos_des[4],traj.pos_des[5],traj.pos_des[6]};
			// command_msg.joint_commands.position = pos_des;
			// std::vector<double> vel_des {traj.vel_des[0], traj.vel_des[1],traj.vel_des[2],traj.vel_des[3],traj.vel_des[4],traj.vel_des[5],traj.vel_des[6]};
			// command_msg.joint_commands.velocity = vel_des;
			// std::vector<double> acc_des {traj.acc_des[0], traj.acc_des[1],traj.acc_des[2],traj.acc_des[3],traj.acc_des[4],traj.acc_des[5],traj.acc_des[6]};
			// command_msg.joint_commands.effort = acc_des;
			// for (int i=0; i<7;i++){
			// 	command_msg.extra_torque[i] = 0.0;
			// }
			// pub_traj.publish(command_msg);

			rate.sleep();
			t = (ros::Time::now() - t_init).toSec();
		}
	}
}

void frankaCallback(const franka_msgs::FrankaStateConstPtr& msg){
	q0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->q).data());
	init_q0 = true;
}

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum){
	cout << "Caught signal " << signum << endl;
	// Terminate program
	exit(signum);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "menu_core");
	ros::NodeHandle node_handle;
	std::string robot_name;// = ROBOT_NAME;
	int SIMULATION = 1;
	int GRIPPER = 2;

	if(!node_handle.getParam("/menu/SIMULATION", SIMULATION))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/menu/GRIPPER", GRIPPER))
		ROS_ERROR("Failed to get parameter from server.");
	std::vector<double> q_tmp;
	if(!node_handle.getParam("/menu/Q0_INIT", q0_init))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/menu/Q0_NAV", q0_nav))
		ROS_ERROR("Failed to get parameter from server.");

	if (SIMULATION){
		robot_name = "";
	}else{
		robot_name = "";
		// robot_name = "/robot/arm";
	}

	ros::Subscriber sub_switch = node_handle.subscribe(robot_name + "/menu/switch_controller", 1, &callback_switch);
	ros::Subscriber sub_franka = node_handle.subscribe<franka_msgs::FrankaState>(robot_name + "/franka_state_controller/franka_states", 1, &frankaCallback);
	ros::Subscriber sub_goto = node_handle.subscribe(robot_name + "/menu/goto", 1, &callback_goto);
	ros::Publisher pub_switch_ack = node_handle.advertise<std_msgs::Int32>(robot_name + "/menu/switch_ack", 1);
	// pub_traj = node_handle.advertise<panda_controllers::Commands>(robot_name + "/computed_torque_controller/command", 1000);

	// SET SLEEP TIME 1000 ---> 1 kHz
	ros::Rate loop_rate(100);

	srand(time(NULL));
	XmlRpc::XmlRpcValue menu_par;

	// Initialize Ctrl-C
	signal(SIGINT, signal_callback_handler);

	while (ros::ok()){
		if (switch_flag != 0){
			// - Swich controller - //
			bool switch_controller_check = 0;
			if (switch_flag == 1)
				switch_controller_check = switch_controllers(robot_name, "computed_torque_mod_controller");
			else if (switch_flag == 2)
				switch_controller_check = switch_controllers(robot_name, "CT_mod_controller_OS");
			else if (switch_flag == 3)
				switch_controller_check = switch_controllers(robot_name, "backstepping_controller");
			else if (switch_flag == 4)
				switch_controller_check = switch_controllers(robot_name, "position_joint_trajectory_controller");
			if(!switch_controller_check){
				ROS_ERROR("Unable to switch controllers! Did you check the namespace? Are you switch from and to the right controller?");
			}else{
				ROS_INFO("The switch has been done correctly!");
			}
			// - ack controller switched - //
			std_msgs::Int32 ack_msg;
			ack_msg.data = 1;
			pub_switch_ack.publish(ack_msg);
			switch_flag = 0;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

// ROS Service from Controller Manager to switch controller
bool switch_controllers(std::string robot_name, std::string to_controller){

    // Temporary bool to be returned
    bool success = false;
	std::string from_controller;
	bool ok_switch = false;

    // Clearing the switch message
    switch_controller.request.start_controllers.clear();
    switch_controller.request.stop_controllers.clear();

    // Call the ListControllers service and save the response in a variable
    controller_manager_msgs::ListControllers srv;
    bool success_list = ros::service::call<controller_manager_msgs::ListControllers>(robot_name + list_controllers_name, srv);
    
    if (success_list){
    // Loop through the response vector and find the running controller
        for (const auto& controller : srv.response.controller){
            if (controller.state == "running"){
               	// ROS_INFO_STREAM("The running controller is: " << controller.name);
			   	if (controller.name != "franka_state_controller" && controller.name != to_controller){
					from_controller = controller.name;
					switch_controller.request.stop_controllers.push_back(from_controller);
					ok_switch = true;
            	}
			}
        }
    } else {
      ROS_ERROR_STREAM("Failed to call ListControllers service");
    }

    // Swithching controller by calling the service
	bool ret = true;
	switch_controller.request.start_controllers.push_back(to_controller);
	switch_controller.request.strictness = 1;
	ret = ros::service::call<controller_manager_msgs::SwitchController>(robot_name + switch_service_name, switch_controller);

	success_list = ros::service::call<controller_manager_msgs::ListControllers>(robot_name + list_controllers_name, srv);
    if (success_list){
    // Loop through the response vector and find the running controller
        for (const auto& controller : srv.response.controller){
            if (controller.state == "running"){
               ROS_INFO_STREAM("The running controller is: " << controller.name);
            }
        }
    } else {
      ROS_ERROR_STREAM("Failed to call ListControllers service");
    }

	return ret;
}