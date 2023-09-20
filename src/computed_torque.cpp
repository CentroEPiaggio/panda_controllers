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

	double kp1, kp2, kp3, kv1, kv2, kv3;

	if (!node_handle.getParam("kp1", kp1) || 
		!node_handle.getParam("kp2", kp2) ||
		!node_handle.getParam("kp3", kp3) || 
		!node_handle.getParam("kv1", kv1) ||
		!node_handle.getParam("kv2", kv2) ||
		!node_handle.getParam("kv3", kv3)) {
		ROS_ERROR("Computed Torque: Could not get parameter kpi or kv!");
		return false;
	}

	Kp = Eigen::MatrixXd::Identity(7, 7);
	Kp(0,0) = kp1; 
	Kp(1,1) = kp1; 
	Kp(2,2) = kp1; 
	Kp(3,3) = kp1; 
	Kp(4,4) = kp2; 
	Kp(5,5) = kp2; 
	Kp(6,6) = kp3;
	
	Kv = Eigen::MatrixXd::Identity(7, 7);
	Kv(0,0) = kv1; 
	Kv(1,1) = kv1; 
	Kv(2,2) = kv1; 
	Kv(3,3) = kv1; 
	Kv(4,4) = kv2; 
	Kv(5,5) = kv2; 
	Kv(6,6) = kv3;
	
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
	
	/* Assigning inertial parameters for initial guess of panda parameters to compute dynamics with regressor */

	for(int i=0; i<NJ; i++){
		double mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz;
		if (!node_handle.getParam("link"+std::to_string(i+1)+"REG"+"/mass", mass) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"REG"+"/m_CoM_x", cmx) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"REG"+"/m_CoM_y", cmy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"REG"+"/m_CoM_z", cmz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"REG"+"/Ixx", xx) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"REG"+"/Ixy", xy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"REG"+"/Ixz", xz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"REG"+"/Iyy", yy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"REG"+"/Iyz", yz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"REG"+"/Izz", zz)){
			
			ROS_ERROR("Computed_torque: Error in parsing inertial parameters!");
			return 1;
		}
		param.segment(PARAM*i, PARAM) << mass,cmx,cmy,cmz,xx,xy,xz,yy,yz,zz;
	}
	std::cout<<"\ninit param:\n"<<param<<"\n";
	regrob::reg2dyn(NJ,PARAM,param,param_dyn);
	std::cout<<"\ninit param_dyn:\n"<<param_dyn<<"\n";

	/* Inizializing the R gains to update parameters*/
	
	std::vector<double> gainRlinks(NJ), gainRparam(3), gainLambda(6);
	Eigen::Matrix<double,PARAM,PARAM> Rlink;

	if (!node_handle.getParam("gainRlinks", gainRlinks) ||
		!node_handle.getParam("gainRparam", gainRparam) ||
		!node_handle.getParam("update_param", update_param_flag)) {
	
		ROS_ERROR("Computed_torque Could not get gain parameter for Lambda, R, Kd!");
		return false;
	}

	Rlink.setZero();
	Rlink(0,0) = gainRparam[0];
	Rlink(1,1) = gainRparam[1];
	Rlink(2,2) = Rlink(1,1);
	Rlink(3,3) = Rlink(1,1);
	Rlink(4,4) = gainRparam[2];
	Rlink(5,5) = Rlink(4,4);
	Rlink(6,6) = Rlink(4,4);
	Rlink(7,7) = Rlink(4,4);
	Rlink(8,8) = Rlink(4,4);
	Rlink(9,9) = Rlink(4,4);

	Rinv.setZero();

	if (update_param_flag){
		for (int i = 0; i<NJ; i++){	
			Rinv.block(i*PARAM, i*PARAM, PARAM, PARAM) = gainRlinks[i]*Rlink;
		}
	}

	/* initialize matrices for updating law*/
	B.setZero();
	B.block(8,0,13,6).setIdentity();
	P.setIdentity(); // soluzione equazione di Lyapunov T.C. (o T.D. ???)
	
	/* Initialize joint (torque,velocity) limits */
	tau_limit << 87, 87, 87, 87, 12, 12, 12;
	q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

	/*Start command subscriber */
	this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState> ("command", 1, &ComputedTorque::setCommandCB, this);   //it verify with the callback that the command has been received
	this->pub_err_ = node_handle.advertise<sensor_msgs::JointState> ("tracking_error", 1);
	this->pub_config_ = node_handle.advertise<panda_controllers::point> ("current_config", 1);
	
	/* Initialize regressor object */
	fastRegMat.init(NJ);

	return true;
}

void ComputedTorque::starting(const ros::Time& time)
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
	command_q_d_old = q_curr;

	command_dot_q_d = dot_q_curr;
	command_dot_q_d_old = dot_q_curr;

	command_dot_dot_q_d.setZero();

	/* Defining the NEW gains */
	Kp_apix = Kp;
	Kv_apix = Kv;

	/* Update regressor */
	dot_q_curr_old.setZero();
	ddot_q_curr.setZero();
	dot_param.setZero();
	fastRegMat.setInertialParam(param_dyn);
    fastRegMat.setArguments(q_curr, dot_q_curr, dot_q_curr, ddot_q_curr);
}

void ComputedTorque::update(const ros::Time&, const ros::Duration& period)
{
	franka::RobotState robot_state = state_handle_->getRobotState();

	std::array<double, 49> mass_array = model_handle_->getMass();
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis();


	M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
	C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
	G = Eigen::Map<Eigen::Matrix<double, 7, 1>> (model_handle_->getGravity().data());
	
	/* =============================================================================== */
	/* check matrix */
	/*
	fastRegMat.setArguments(q_curr,dot_q_curr,param_dyn);
	Mest = fastRegMat.getMass_gen();
	Cest = fastRegMat.getCoriolis_gen();
	Gest = fastRegMat.getGravity_gen();

	std::cout<<"\n ros Mass:\n"<<M<<"\n";
	std::cout<<"\n est Mass:\n"<<Mest<<"\n";
	std::cout<<"\n ros Coriolis:\n"<<C<<"\n";
	std::cout<<"\n est Coriol:\n"<<Cest*dot_q_curr<<"\n";
	*/
/* =============================================================================== */
	
	/* Actual position and velocity of the joints */

	T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
	q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
	ddot_q_curr = (dot_q_curr - dot_q_curr_old) / dt;
	dot_q_curr_old = dot_q_curr;

	/* tau_J_d is the desired link-side joint torque sensor signals without gravity */

	tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

	/* Saturate desired velocity to avoid limits */
	for (int i = 0; i < 7; ++i){
		double ith_des_vel = abs(command_dot_q_d(i)/q_dot_limit(i));
		if( ith_des_vel > 1)
		command_dot_q_d = command_dot_q_d / ith_des_vel; 
	}

	/* compute joints derired effort */

	// command_dot_dot_q_d = (command_dot_q_d - command_dot_q_d_old) / dt;

	/* Computed Torque control law */

	error = command_q_d - q_curr;
	dot_error = command_dot_q_d - dot_q_curr;
	x.head(7) = error;
	x.tail(7) = dot_error;

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

	/* Update and Compute Regressor */
	
	fastRegMat.setArguments(q_curr, dot_q_curr, dot_q_curr, ddot_q_curr);
	Y = fastRegMat.getReg_gen();

	/* tau_J_d is past tau_cmd saturated */


	/* Update inertial parameters */

	//regrob::reg2dyn(NJ,PARAM,param,param_dyn);
	fastRegMat.setArguments(q_curr,dot_q_curr,param_dyn);
	Mest = fastRegMat.getMass_gen();
	if (update_param_flag){
		dot_param = Rinv*Y.transpose()*Mest.inverse()*B.transpose()*P*x;
		param = param + dt*dot_param;
	}
	
	/* update dynamic for control law */

	regrob::reg2dyn(NJ,PARAM,param,param_dyn);					// conversion of updated parameters
	fastRegMat.setArguments(q_curr,dot_q_curr,param_dyn);
	Mest = fastRegMat.getMass_gen();
	Cest = fastRegMat.getCoriolis_gen();
	Gest = fastRegMat.getGravity_gen();

	tau_cmd = Mest * command_dot_dot_q_d + Cest*dot_q_curr + Kp_apix * error + Kv_apix * dot_error + Gest - G;  // C->C*dq
	
	//std::cout<<"\ntau \n"<<tau_cmd<<std::endl;

	/* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
	tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
	

	/* Set the command for each joint */
	for (size_t i = 0; i < 7; i++) {
		joint_handles_[i].setCommand(tau_cmd[i]);
	}

	msg_config.header.stamp  = ros::Time::now();
	msg_config.xyz.x = T0EE.translation()(0);
	msg_config.xyz.y = T0EE.translation()(1);
	msg_config.xyz.z = T0EE.translation()(2);

	this->pub_config_.publish(msg_config);
}

void ComputedTorque::stopping(const ros::Time&)
{
	//TO DO
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

void ComputedTorque::setCommandCB(const sensor_msgs::JointState::ConstPtr& msg)
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

}

PLUGINLIB_EXPORT_CLASS(panda_controllers::ComputedTorque, controller_interface::ControllerBase);
