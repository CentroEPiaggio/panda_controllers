//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/computed_torque_adaptive.h> //library of the computed torque 

namespace panda_controllers{

bool ComputedTorqueAdaptive::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{ 
	this->cvc_nh = node_handle;
	
	std::string arm_id; //checking up the arm id of the robot
	if (!node_handle.getParam("arm_id", arm_id)) {
		ROS_ERROR("Computed Torque: Could not get parameter arm_id!");
		return false;
	}

	/* Inizializing the Kp and Kv gains */

	double kp1, kp2, kp3, kv1, kv2, kv3;

	if (!node_handle.getParam("kp1", kp1) || !node_handle.getParam("kp2", kp2) || !node_handle.getParam("kp3", kp3) 
		|| !node_handle.getParam("kv1", kv1) || !node_handle.getParam("kv2", kv2) || !node_handle.getParam("kv3", kv3)) {
		ROS_ERROR("Computed Torque: Could not get parameter kpi or kv!");
		return false;
	}

	Kp = Eigen::MatrixXd::Identity(7, 7);
	Kp(0,0) = kp1; Kp(1,1) = kp1; Kp(2,2) = kp1; Kp(3,3) = kp1; Kp(4,4) = kp2; Kp(5,5) = kp2; Kp(6,6) = kp3;
	
	Kv = Eigen::MatrixXd::Identity(7, 7);
	Kv(0,0) = kv1; Kv(1,1) = kv1; Kv(2,2) = kv1; Kv(3,3) = kv1; Kv(4,4) = kv2; Kv(5,5) = kv2; Kv(6,6) = kv3;
	
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
	
    for(int i=0; i<7; i++){

		double mass, xx, xy, xz, yy, yz, zz;
		std::vector<double> distCM;
		
		distCM.resize(3);
		
		if (!node_handle.getParam("link"+std::to_string(i+1)+"/mass", mass) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"/m_origin/xyz", distCM) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"/inertia/xx", xx) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"/inertia/xy", xy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"/inertia/xz", xz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"/inertia/yy", yy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"/inertia/yz", yz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"/inertia/zz", zz)){
				
			ROS_ERROR("Backstepping: Error in parsing inertial parameters!");
			return 1;
		}

		param.segment(10*i, 10) << mass,distCM[0],distCM[1],distCM[2],xx,xy,xz,yy,yz,zz;
	}

	/* Initialize joint (torque,velocity) limits */
	tau_limit << 87, 87, 87, 87, 12, 12, 12;
	q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

	/*Start command subscriber */
	this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState> ("command", 1, &ComputedTorqueAdaptive::setCommandCB, this);   //it verify with the callback that the command has been received
	this->pub_err_ = node_handle.advertise<sensor_msgs::JointState> ("tracking_error", 1);
	this->pub_err_ = node_handle.advertise<panda_controllers::log_backstepping> ("logging", 1);
	this->pub_config_ = node_handle.advertise<panda_controllers::point> ("current_config", 1);

	const int nj = 7;
	fastRegMat.init(nj);

	return true;
}

void ComputedTorqueAdaptive::starting(const ros::Time& time)
{
	/* Getting Robot State in order to get q_curr and dot_q_curr */
	franka::RobotState robot_state = state_handle_->getRobotState();

	std::array<double, 49> mass_array = model_handle_->getMass();
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

	/* Mapping actual joints position, actual joints velocity, Mass matrix and Coriolis vector onto Eigen form  */
	q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

	M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());

	/* Secure Initialization */
	command_q_d = q_curr;
	//command_q_d << M_PI_2,-0.7855,0,-2.3561,0,M_PI_2,0.7854;
	command_q_d << 0, -0.785398163, 0, -2.35619449, 0, 1.57079632679, 0.785398163397;
	command_q_d_old = q_curr;

	command_dot_q_d = dot_q_curr;
	command_dot_q_d_old = dot_q_curr;

	command_dot_dot_q_d.setZero();

	/* Defining the NEW gains */
	Kp_apix = Kp;
	Kv_apix = Kv;

    /* Update regressor */

	dot_param.setZero();
    fastRegMat.setArguments(q_curr, dot_q_curr, dot_q_curr, command_dot_dot_q_d);

}

void ComputedTorqueAdaptive::update(const ros::Time&, const ros::Duration& period)
{
	franka::RobotState robot_state = state_handle_->getRobotState();

	std::array<double, 49> mass_array = model_handle_->getMass();
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
	Eigen::Affine3d T0EE;

	M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
    G = Eigen::Map<Eigen::Matrix<double, 7, 1>> (model_handle_->getGravity().data());
	
	T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());

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

	// command_dot_dot_q_d = (command_dot_q_d - command_dot_q_d_old) / dt;

	/* Computed Torque control law */

	error = command_q_d - q_curr;
	dot_error = command_dot_q_d - dot_q_curr;

	// Publish tracking errors as joint states
	sensor_msgs::JointState error_msg;
	std::vector<double> err_vec(error.data(), error.data() + error.rows()*error.cols());
	std::vector<double> dot_err_vec(dot_error.data(), dot_error.data() + dot_error.rows()*dot_error.cols());
	error_msg.header.stamp = ros::Time::now();
	error_msg.position = err_vec;
	error_msg.velocity = dot_err_vec;
	this->pub_err_.publish(error_msg);

	Kp_apix = Kp;
	Kv_apix = Kv;

	tau_cmd = M * command_dot_dot_q_d + C + Kp_apix * error + Kv_apix * dot_error;  // C->C*dq


	/* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
	tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
	
	time_now = ros::Time::now();
	msg_log.header.stamp = time_now;
	fillMsg(msg_log.Link1_param, param.segment(0, 10));
	fillMsg(msg_log.Link2_param, param.segment(10, 10));
	fillMsg(msg_log.Link3_param, param.segment(20, 10));
	fillMsg(msg_log.Link4_param, param.segment(30, 10));
	fillMsg(msg_log.Link5_param, param.segment(40, 10));
	fillMsg(msg_log.Link6_param, param.segment(50, 10));
	fillMsg(msg_log.Link7_param, param.segment(60, 10));
	fillMsg(msg_log.tau_cmd, tau_cmd);

	msg_config.header.stamp  = time_now;
	msg_config.xyz.x = T0EE.translation()(0);
	msg_config.xyz.y = T0EE.translation()(1);
	msg_config.xyz.z = T0EE.translation()(2);

	this->pub_err_.publish(msg_log);
	this->pub_config_.publish(msg_config);

	/* Set the command for each joint */
	for (size_t i = 0; i < 7; i++) {
		joint_handles_[i].setCommand(tau_cmd[i]);
	}
}

void ComputedTorqueAdaptive::stopping(const ros::Time&)
{
	//TO DO
}

/* Check for the effort commanded */
Eigen::Matrix<double, 7, 1> ComputedTorqueAdaptive::saturateTorqueRate(
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

void ComputedTorqueAdaptive::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)
{
	if ((msg->position).size() != 7 || (msg->position).empty()) {

		ROS_FATAL("Desired position has not dimension 7 or is empty!", (msg->position).size());
	}

	if ((msg->velocity).size() != 7 || (msg->velocity).empty()) {

		ROS_FATAL("Desired velocity has not dimension 7 or is empty!", (msg->velocity).size());
	}

	// TODO: Here we assign acceleration to effort (use trajectory_msgs::JointTrajectoryMessage)
	if ((msg->effort).size() != 7 || (msg->effort).empty()) {

		ROS_FATAL("Desired effort (acceleration) has not dimension 7 or is empty!", (msg->effort).size());
	}

	command_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
	command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->velocity).data());
	command_dot_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->effort).data());

}

template <size_t N>
void ComputedTorqueAdaptive::fillMsg(boost::array<double, N>& msg_, const Eigen::VectorXd& data_) {
    
	int dim = data_.size();
    for (int i = 0; i < dim; i++) {
        msg_[i] = data_[i];
    }
}
}

PLUGINLIB_EXPORT_CLASS(panda_controllers::ComputedTorqueAdaptive, controller_interface::ControllerBase);
