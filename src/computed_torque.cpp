//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/computed_torque.h> //library of the computed torque 

namespace panda_controllers{

bool ComputedTorque::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{ 
	this->cvc_nh = node_handle;

	std::string arm_id; //checking up the arm id of the robot
	if (!node_handle.getParam("arm_id", arm_id)) {
		ROS_ERROR("Computed Torque: Could not get parameter arm_id!");
		return false;
	}

	/* Inizializing the Kp and Kv gains */

	double kp1, kp2, kp3, kp4, kv1, kv2, kv3, kv4;

	if (!node_handle.getParam("kp1", kp1) || !node_handle.getParam("kp2", kp2) || !node_handle.getParam("kp3", kp3) 
		|| !node_handle.getParam("kv1", kv1) || !node_handle.getParam("kv2", kv2) || !node_handle.getParam("kv3", kv3)) {
		ROS_ERROR("Computed Torque: Could not get parameter kpi or kv!");
		return false;
	}
	if (!node_handle.getParam("wrist_stiffness", wrist_stiffness)) {
		ROS_ERROR("ILC controller: Could not get wrist_stiffness parameter!");
		return false;
	}
	if (!node_handle.getParam("wrist_offset", wrist_offset)) {
		ROS_ERROR("ILC controller: Could not get wrist_offset parameter!");
		return false;
	}
	if (!node_handle.getParam("wrist_fixed", wrist_fixed)) {
		ROS_ERROR("ILC controller: Could not get wrist_fixed parameter!");
		return false;
	}
	if (!node_handle.getParam("wrist_theta", wrist_theta)) {
		ROS_ERROR("ILC controller: Could not get wrist_theta parameter!");
		return false;
	}

		Kp = Eigen::MatrixXd::Identity(8, 8);
		Kp(0,0) = kp1; Kp(1,1) = kp1; Kp(2,2) = kp1; Kp(3,3) = kp1; Kp(4,4) = kp2; Kp(5,5) = kp2; Kp(6,6) = kp3; Kp(7,7) = kp4;

		Kv = Eigen::MatrixXd::Identity(8, 8);
		Kv(0,0) = kv1; Kv(1,1) = kv1; Kv(2,2) = kv1; Kv(3,3) = kv1; Kv(4,4) = kv2; Kv(5,5) = kv2; Kv(6,6) = kv3; Kv(7,7) = kv4;

	/* Assigning the time */

	if (!node_handle.getParam("dt", dt)) {
		ROS_ERROR("Computed Torque: Could not get parameter dt!");
		return false;
	}

	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		ROS_ERROR("Computed Torque: Error in parsing joints name!");
		return false;
	}

	franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
		ROS_ERROR_STREAM("Computed Torque: Error getting model interface from hardware!");
		return false;
	}

	try {
		model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM("Computed Torque: Exception getting model handle from interface: " << ex.what());
		return false;
	}

	franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
		ROS_ERROR_STREAM("Computed Torque: Error getting state interface from hardware");
		return false;
	}

	try {
		state_handle_.reset(new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM("Computed Torque: Exception getting state handle from interface: " << ex.what());
		return false;
	}

	hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	if (effort_joint_interface == nullptr) {
		ROS_ERROR_STREAM("Computed Torque: Error getting effort joint interface from hardware!");
		return false;
	}

	for (size_t i = 0; i < 7; ++i) {
		try {
			joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			ROS_ERROR_STREAM("Computed Torque: Exception getting joint handles: " << ex.what());
			return false;
		}
	}

	/* Initialize joint (torque,velocity) limits */
	tau_limit << 87, 87, 87, 87, 12, 12, 12;
	q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

	/* Diagonal matrix for damping */
	damping.setZero();
	damping.diagonal() << 10, 50, 50, 20, 10, 10, 10, 0;    // 0.15 for qbmove

	/* Soft wrist constants */
	a1 = 8.9995;
	k1 = 0.0026;
	a2 = 8.9989;
	k2 = 0.0011;

	// /* Set to false the switch_control flag: robot starts with computed torque controller */
	// switch_control = false;


	/* Initialize soft joint state */
	sensor_msgs::JointState::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/cube_wrist/qbmove2/control/joint_states", node_handle);
	if(msg) {
		Eigen::MatrixXd state_vector = Eigen::Map<const Eigen::Matrix<double,4,1>>((msg->position).data()); // current joint position
		theta1 = state_vector(0);       // theta1 motor angle
		theta2 = state_vector(1);       // theta2 motor angle
		q_wrist(7) = state_vector(2);   // joint angle
		Eigen::MatrixXd velocity_vector = Eigen::Map<const Eigen::Matrix<double,4,1>>((msg->velocity).data()); // current joint velocity
		dq_wrist(7) = velocity_vector(2);   // joint velocity
	}

	// get absolute path to frankawrist_inertial_REG.yaml file
	std::string package_path = ros::package::getPath("panda_controllers");
	package_path += "/config/frankawrist_inertial_REG.yaml";
	// load inertial parameters from file
	franka_robot.load_inertial_REG(package_path);

	ros::Rate loopRate(10);
	loopRate.sleep();

	/* Start subscribers nodes */
	// sub_command: read the desired joint position, velocity and trajectory
	this->sub_command = node_handle.subscribe<sensor_msgs::JointState> ("command", 1, &ComputedTorque::setCommandCB, this);
	// // sub_flag: read the flag set by torque_signal node to switch between controllers
	// this->sub_flag = node_handle.subscribe<std_msgs::Bool> ("trajectory_tracking", 1, &ComputedTorque::setFlagCB, this);
	// sub_soft_joints: read joint states from wrist soft joint
	this->sub_soft_joints = node_handle.subscribe<sensor_msgs::JointState> ("/cube_wrist/qbmove2/control/joint_states", 1, &ComputedTorque::setJointState, this);
	// sub_command_wrist: read the desired joint position, velocity and trajectory for Franka + soft wrist
	this->sub_command_wrist = node_handle.subscribe<sensor_msgs::JointState> ("command_wrist", 1, &ComputedTorque::setCommandWristCB, this);
	// // sub_gotostart: to move the cube to the initial position
	// this->sub_gotostart = node_handle.subscribe<std_msgs::Bool> ("gotostart", 1, &ComputedTorque::setGoToStartCB, this);
	
	/* Start publishers nodes */
	// pub_err: to track the errors on the robot's configuration
	this->pub_err_ = node_handle.advertise<sensor_msgs::JointState> ("tracking_error", 100, true);
	// // pub_effort_cmd: to track the actual torque (effort) applied at the robot's actuators
	// this->pub_effort_cmd = node_handle.advertise<std_msgs::Float64MultiArray> ("effort_command",1000, true);
	// // pub_config: to track the real robot's configuration and its derivative
	// this->pub_config = node_handle.advertise<sensor_msgs::JointState> ("robot_configuration", 1);
	// // pub_reset: to reset simulation
	// this->pub_reset = node_handle.advertise<std_msgs::Bool> ("reset_simulation", 1, true);
	// // pub_gravity: to publish gravity vector
	// this->pub_gravity = node_handle.advertise<std_msgs::Float64MultiArray>("gravity_vector", 1000);
	// pub_softcommand: to send controller signal to soft joint
	this->pub_softcommand = node_handle.advertise<trajectory_msgs::JointTrajectory>("/cube_wrist/qbmove2/control/qbmove2_position_and_preset_trajectory_controller/command", 1);
	// pub_stiffness: to see the current soft joint stiffness
	this->pub_stiffness = node_handle.advertise<std_msgs::Float64>("cube_wrist_stiffness", 1);
	// pub_thetacontrol: to see the current shaft position for sot joint
	this->pub_thetacontrol = node_handle.advertise<std_msgs::Float64>("cube_wrist_thetacontrol", 1);

	return true;
}

void ComputedTorque::starting(const ros::Time& time){
	/* Getting Robot State in order to get q_curr and dot_q_curr */
	franka::RobotState robot_state = state_handle_->getRobotState();

	// std::array<double, 49> mass_array = model_handle_->getMass();
	// std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
	// M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	// C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());

	/* Mapping actual joints position, actual joints velocity, Mass matrix and Coriolis vector onto Eigen form  */
	q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

	/* Secure Initialization */
	command_q_d = q_curr;
	command_q_d_old = q_curr;

	command_dot_q_d = dot_q_curr;
	command_dot_q_d_old = dot_q_curr;

	command_dot_dot_q_d.setZero();

	/* Defining the NEW gains */
	Kp_apix = Kp;
	Kv_apix = Kv;

	/* Franka + qbmove */
	// Current joint initialization
	q_wrist.head<7>() = q_curr;
	dq_wrist.head<7>() = dot_q_curr;
	franka_robot.setArguments(q_wrist, dq_wrist, Eigen::Matrix<double, 8, 1>::Zero(), Eigen::Matrix<double, 8, 1>::Zero());
	// Desired joint values initialization
	q_des_wrist = q_wrist;
	dq_des_wrist.setZero();
	ddq_des_wrist.setZero();
	G_Wrist = franka_robot.getGravity();
	std::cout << "Gravity vector thunder: " << std::endl;
	std::cout << G_Wrist.transpose() << std::endl;
	/* Franka gravity vector */
	std::array<double, 7> gravity_vector = model_handle_->getGravity();
	G = Eigen::Map<Eigen::Matrix<double, 7, 1>>(gravity_vector.data());
	std::cout << "Gravity vector libfranka: " << std::endl;
	std::cout << G.transpose() << std::endl;

	// initializing wrist
	if (wrist_fixed){
		theta_control = wrist_theta;
		trajectory_msgs::JointTrajectory qbmove_command;
		trajectory_msgs::JointTrajectoryPoint qbmove_point;
		qbmove_command.header.stamp = ros::Time::now();
		qbmove_command.joint_names.push_back("qbmove2_shaft_joint");                    // shaft position
		qbmove_command.joint_names.push_back("qbmove2_stiffness_preset_virtual_joint"); // stiffness preset
		qbmove_point.positions.push_back(wrist_theta);    // shaft position [rad]
		qbmove_point.positions.push_back(wrist_stiffness);  // stiffness preset [0,1]
		qbmove_point.time_from_start = ros::Duration(1.0);
		// Add point to trajectory message
		qbmove_command.points.push_back(qbmove_point);
		// Send message to the qbmove wrist
		pub_softcommand.publish(qbmove_command);
	}
}

void ComputedTorque::update(const ros::Time&, const ros::Duration& period){
	franka::RobotState robot_state = state_handle_->getRobotState();

	std::array<double, 49> mass_array = model_handle_->getMass();
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
	std::array<double, 7> gravity_vector = model_handle_->getGravity();

	// M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	// C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());	
    G = Eigen::Map<Eigen::Matrix<double, 7, 1>>(gravity_vector.data());
	
	/* Actual position and velocity of the joints */

	q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

	/* tau_J_d is the desired link-side joint torque sensor signals without gravity */
	tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

	/* Saturate desired velocity to avoid limits */
	for (int i = 0; i < 7; ++i){
		double ith_des_vel = abs(command_dot_q_d(i)/q_dot_limit(i));
		if( ith_des_vel > 1)
		command_dot_q_d = command_dot_q_d / ith_des_vel; 
	}

	error = command_q_d - q_curr;
	dot_error = command_dot_q_d - dot_q_curr;

	Kp_apix = Kp;
	Kv_apix = Kv;

	/* tau_J_d is the desired link-side joint torque sensor signals without gravity */
	tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

	/* thunder (8giunti)*/
	// Get current robot configuration and velocity
	q_wrist.head<7>() = q_curr;
	dq_wrist.head<7>() = dot_q_curr;
	// Get current mass and coriolis matrices and gravity vector
	franka_robot.setArguments(q_wrist, dq_wrist, Eigen::Matrix<double, 8, 1>::Zero(), Eigen::Matrix<double, 8, 1>::Zero());
	M_Wrist = franka_robot.getMass();
	C_Wrist = franka_robot.getCoriolis();
	G_Wrist = franka_robot.getGravity();
	// Compute current position and velocity error
	error_wrist = q_des_wrist - q_wrist;
	dot_error_wrist = dq_des_wrist - dq_wrist;

	// ----- Computed torque controller ----- //
	/* Only for Franka */
	// tau_cmd = M * command_dot_dot_q_d + C + Kp_apix.block<7,7>(0,0) * error + Kv_apix.block<7,7>(0,0) * dot_error;  // C->C*dq;
	// tau_cmd = M * command_dot_dot_q_d + C + G + Kp_apix * error + Kv_apix * dot_error;  // Apply computed torque law + payload compensation

	/* Franka + soft wrist */
	tau_cmd_wrist = M_Wrist*ddq_des_wrist + C_Wrist*dq_wrist + G_Wrist + Kp_apix*error_wrist + Kv_apix*dot_error_wrist;

	/* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
	tau_cmd = tau_cmd_wrist.head<7>()-G;

	tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);

	/* Compute qbmove stiffness */
	// stiffness = a1*k1*cosh(a1*(q_wrist(7)-theta1)) + a2*k2*cosh(a2*(q_wrist(7)-theta2));
	stiffness = wrist_stiffness * 2.3 / 0.4; 	// phisical stiffness driven by the imposed one [0-1]

	if (wrist_fixed){
		theta_control = wrist_theta;
	} else {
		/*  Compute control action for qbmove joint -> theta = q + K^-1 * tau */
		// theta_control = (q_wrist(7) - wrist_offset) + (1/stiffness) * tau_cmd_wrist(7);
		theta_control = (q_des_wrist(7) - wrist_offset) + (1/stiffness) * tau_cmd_wrist(7);
	}

	/* Set the command for each rigid joint... */
	for (size_t i = 0; i < 7; i++) {
		// joint_handles_[i].setCommand(tau_cmd_wrist[i]);	// send command to the robot's i-th joint 
		joint_handles_[i].setCommand(tau_cmd[i]);
		// Fill gravity message
		// gravity_msg.data[i] = G_Wrist[i];
	}
	/* ...and for soft joint */
	// gravity_msg.data[7] = G_Wrist(7);
	// Publish gravity vector message
	// this->pub_gravity.publish(gravity_msg);

	/* Set the command for qbmove soft joint using position and preset trajectory controller */
	if (!wrist_fixed) {
		trajectory_msgs::JointTrajectory qbmove_command;
		trajectory_msgs::JointTrajectoryPoint qbmove_point;
		qbmove_command.header.stamp = ros::Time::now();
		qbmove_command.joint_names.push_back("qbmove2_shaft_joint");                    // shaft position
		qbmove_command.joint_names.push_back("qbmove2_stiffness_preset_virtual_joint"); // stiffness preset
		qbmove_point.positions.push_back(theta_control);    // shaft position [rad]
		qbmove_point.positions.push_back(wrist_stiffness);  // stiffness preset [0,1]
		qbmove_point.time_from_start = ros::Duration(0.1); // check this ------------------------------------------------------------------------------!
		// Add point to trajectory message
		qbmove_command.points.push_back(qbmove_point);
		// Send message to the qbmove wrist
		pub_softcommand.publish(qbmove_command);
	}

	/* Publish theta_control value */
	std_msgs::Float64 thetacontrol_msg;
	thetacontrol_msg.data = theta_control;
	this->pub_thetacontrol.publish(thetacontrol_msg);
	
	/* Publish current stiffness value */
	std_msgs::Float64 stiffness_msg;
	stiffness_msg.data = wrist_stiffness;
	this->pub_stiffness.publish(stiffness_msg);

	/* Publish messages to track torque and gravity */
	// this->pub_gravity.publish(gravity_msg);

	/* Publish current robot configuration and its derivative */
	// sensor_msgs::JointState config_msg;
	// std::vector<double> q_vect(q_wrist.data(),q_wrist.data()+q_wrist.size());
	// std::vector<double> dq_vect(dq_wrist.data(),dq_wrist.data()+dq_wrist.size());
	// config_msg.header.stamp = ros::Time::now();
	// config_msg.position = q_vect;
	// config_msg.velocity = dq_vect;
	// pub_config.publish(config_msg);

}


void ComputedTorque::stopping(const ros::Time&)
{
	// TO-DO
}

/* Check for the effort commanded */
Eigen::Matrix<double, 7, 1> ComputedTorque::saturateTorqueRate(
	const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
	const Eigen::Matrix<double, 7, 1>& tau_J_d)
{
	Eigen::Matrix<double, 7, 1> tau_d_saturated {};
	for (size_t i = 0; i < 7; i++) {

		double difference = tau_d_calculated[i] - tau_J_d[i];
		tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);

	}
	return tau_d_saturated;
}

void ComputedTorque::setCommandCB(const sensor_msgs::JointStateConstPtr& msg){
	if ((msg->position).size() != 7 || (msg->position).empty()) {
		ROS_FATAL("Desired position has not dimension 7 or is empty! Position vector size: %i", int((msg->position).size()));
	}

	if ((msg->velocity).size() != 7 || (msg->velocity).empty()) {

		ROS_FATAL("Desired velocity has not dimension 7 or is empty! Velocity vector size: %i", int((msg->velocity).size()));
	}

	// TODO: Here we assign acceleration to effort (use trajectory_msgs::JointTrajectoryMessage)
	if ((msg->effort).size() != 7 || (msg->effort).empty()) {

		ROS_FATAL("Desired effort (acceleration) has not dimension 7 or is empty! Effort vector size: %i", int((msg->effort).size()));
	}

	command_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
	command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->velocity).data());
	command_dot_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->effort).data());
}
 // Callback function for sub_soft_joints subscriber node
void ComputedTorque::setJointState(const sensor_msgs::JointStateConstPtr& msg) {
	Eigen::MatrixXd state_vector = Eigen::Map<const Eigen::Matrix<double,4,1>>((msg->position).data()); // current joint position
	theta1 = state_vector(0);       // theta1 motor angle
	theta2 = state_vector(1);       // theta2 motor angle
	q_wrist(7) = state_vector(2);   // joint angle
	Eigen::MatrixXd velocity_vector = Eigen::Map<const Eigen::Matrix<double,4,1>>((msg->velocity).data()); // current joint velocity
	dq_wrist(7) = velocity_vector(2);   // joint velocity
}

// Callback function for sub_command_wrist subscriber node
void ComputedTorque::setCommandWristCB (const sensor_msgs::JointStateConstPtr& msg) {
	if ((msg->position).size() != 8 || (msg->position).empty()) {
		ROS_FATAL("Desired position has not dimension 8 or is empty! Position vector size: %i", int((msg->position).size()));
	}

	if ((msg->velocity).size() != 8 || (msg->velocity).empty()) {

		ROS_FATAL("Desired velocity has not dimension 8 or is empty! Velocity vector size: %i", int((msg->velocity).size()));
	}

	// TODO: Here we assign acceleration to effort (use trajectory_msgs::JointTrajectoryMessage)
	if ((msg->effort).size() != 8 || (msg->effort).empty()) {

		ROS_FATAL("Desired effort (acceleration) has not dimension 8 or is empty! Effort vector size: %i", int((msg->effort).size()));
	}

	q_des_wrist = Eigen::Map<const Eigen::Matrix<double, 8, 1>>((msg->position).data());
	dq_des_wrist = Eigen::Map<const Eigen::Matrix<double, 8, 1>>((msg->velocity).data());
	ddq_des_wrist = Eigen::Map<const Eigen::Matrix<double, 8, 1>>((msg->effort).data());
}

}

PLUGINLIB_EXPORT_CLASS(panda_controllers::ComputedTorque, controller_interface::ControllerBase);
