//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/backstepping.h> //library of the Backstepping 

namespace panda_controllers{

bool Backstepping::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{ 
	this->cvc_nh = node_handle;
	
	/* Init setup of arm */

	std::string arm_id;
	if (!node_handle.getParam("arm_id", arm_id)) {
		ROS_ERROR("Backstepping: Could not get parameter arm_id!");
		return false;
	}

	franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
		ROS_ERROR_STREAM("Backstepping: Error getting model interface from hardware!");
		return false;
	}
	try {
		model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM("Backstepping: Exception getting model handle from interface: " << ex.what());
		return false;
	}

	franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
		ROS_ERROR_STREAM("Backstepping: Error getting state interface from hardware");
		return false;
	}
	try {
		state_handle_.reset(new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM("Backstepping: Exception getting state handle from interface: " << ex.what());
		return false;
	}

	hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	if (effort_joint_interface == nullptr) {
		ROS_ERROR_STREAM("Backstepping: Error getting effort joint interface from hardware!");
		return false;
	}

	/* Inizializing the Lambda and R and Kd gains */
	
	double gainLambda, gainR, gainKd, gainKn;
	if (!node_handle.getParam("gainLambda", gainLambda) ||
		!node_handle.getParam("gainR", gainR) ||
		!node_handle.getParam("gainKd", gainKd)||
		!node_handle.getParam("update_param", update_param_flag)) {
		ROS_ERROR("Backstepping: Could not get gain parameter for Lambda, R, Kd!");
		return false;
	}

	Lambda = gainLambda * Eigen::Matrix3d::Identity();
	R = gainR * Eigen::MatrixXd::Identity(70, 70);
	if(R.determinant()!=0){
		Rinv = R.inverse();
	}else{
		update_param_flag = false;
	}

	Kd = gainKd * Eigen::MatrixXd::Identity(7, 7);

	/* Assigning joint names */

	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		ROS_ERROR("Backstepping: Error in parsing joints name!");
		return false;
	}
	for (size_t i = 0; i < 7; ++i) {
		try {
			joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			ROS_ERROR_STREAM("Backstepping: Exception getting joint handles: " << ex.what());
			return false;
		}
	}
	
	/* Assigning inertial parameters for initial guess of panda parameters to compute dynamics */

	std::vector<Eigen::VectorXd> link_vector(7);

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
	q_max_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
	q_min_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
	q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

	/*Start command subscriber and advertise */

	this->sub_command_ = node_handle.subscribe<panda_controllers::desTrajEE> ("command", 1, &Backstepping::setCommandCB, this);   //it verify with the callback that the command has been received
	this->pub_err_ = node_handle.advertise<panda_controllers::log_backstepping> ("logging", 1);
	this->pub_config_ = node_handle.advertise<geometry_msgs::Point> ("current_config", 1);
	
	/* Initialize regressor object */

	const int nj = 7;
	fastRegMat.init(nj);

	return true;
}

void Backstepping::starting(const ros::Time& time)
{	
	/* Getting Robot State in order to get q_curr and dot_q_curr and jacobian of end-effector */
	
	franka::RobotState robot_state = state_handle_->getRobotState();
	Eigen::Affine3d T0EE(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

	/* Secure initialization command */

	ee_pos_cmd = T0EE.translation();
	ee_vel_cmd.setZero();
	ee_acc_cmd.setZero();

	/* Compute error */

	error.setZero();
	dot_error.setZero();
	
	/* Compute reference (Position Control) */
	
	dot_qr.setZero();
	ddot_qr.setZero();
	
	/* Update regressor */

	dot_param.setZero();
    fastRegMat.setArguments(q_curr, dot_q_curr, dot_qr, ddot_qr);
}

void Backstepping::update(const ros::Time&, const ros::Duration& period)
{

	/* Getting Robot State in order to get q_curr and dot_q_curr and jacobian of end-effector */
	
	franka::RobotState robot_state = state_handle_->getRobotState();

	Eigen::Matrix<double, 6, 7> jacobian;
	Eigen::Matrix<double, 7, 1> G;
	Eigen::Affine3d T0EE;
	
	jacobian = Eigen::Map<Eigen::Matrix<double, 6, 7>>(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());
	G = Eigen::Map<Eigen::Matrix<double, 7, 1>> (model_handle_->getGravity().data());
	
	T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
	q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
	
	/* Update pseudo-inverse of jacobian and its derivative */

	fastRegMat.setArguments(q_curr,dot_q_curr);

	/* Compute pseudo-inverse of jacobian and its derivative */

	Eigen::MatrixXd mypJacEE = fastRegMat.getPinvJac_gen();
	Eigen::MatrixXd mydot_pJacEE = fastRegMat.getDotPinvJac_gen();

	/* Compute error */

	Eigen::Vector3d ee_position = T0EE.translation();
	Eigen::Vector3d ee_velocity = jacobian.topRows(3)*dot_q_curr;

	error = ee_pos_cmd - ee_position;
	dot_error = ee_vel_cmd - ee_velocity;

	/* Compute reference */

	Eigen::Vector3d tmp_position = ee_vel_cmd + Lambda * error;
	Eigen::Vector3d tmp_velocity = ee_acc_cmd + Lambda * dot_error;

	dot_qr = mypJacEE * tmp_position;
	ddot_qr = mypJacEE * tmp_velocity + mydot_pJacEE * tmp_position;
	s = dot_qr - dot_q_curr;
	
	/* Update and Compute Regressor */

	fastRegMat.setArguments(q_curr, dot_q_curr, dot_qr, ddot_qr);
	Yr = fastRegMat.getReg_gen();

	/* tau_J_d is past tau_cmd saturated */

	tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

	/* Backstepping control law */

	dt = period.toSec();

	/* Update inertial parameters */
	
	if (update_param_flag){
		dot_param = Rinv*Yr.transpose()*s;
		param = param + dt*dot_param;
	}

	/* Compute tau command */
	
	tau_cmd = Yr*param + Kd*s + jacobian.topRows(3).transpose()*error-G;

 	/* Publish tracking errors as joint states */

	msg_log.header.stamp = ros::Time::now();
	fillMsg(msg_log.error_pos_EE, error);
	fillMsg(msg_log.dot_error_pos_EE, dot_error);
	fillMsg(msg_log.dot_qr, dot_qr);
	fillMsg(msg_log.ddot_qr, ddot_qr);
	fillMsg(msg_log.s, s);
	fillMsg(msg_log.Link1_param, param.segment(0, 10));
	fillMsg(msg_log.Link2_param, param.segment(10, 10));
	fillMsg(msg_log.Link3_param, param.segment(20, 10));
	fillMsg(msg_log.Link4_param, param.segment(30, 10));
	fillMsg(msg_log.Link5_param, param.segment(40, 10));
	fillMsg(msg_log.Link6_param, param.segment(50, 10));
	fillMsg(msg_log.Link7_param, param.segment(60, 10));
	fillMsg(msg_log.tau_cmd, tau_cmd);

	msg_config.x = T0EE.translation()(0);
	msg_config.y = T0EE.translation()(1);
	msg_config.z = T0EE.translation()(2);

	this->pub_err_.publish(msg_log);
	this->pub_config_.publish(msg_config);

	/* Saturate the derivate of tau_cmd */
	
	tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
	
	/* Set the command for each joint */

	for (size_t i = 0; i < 7; i++) {
		joint_handles_[i].setCommand(tau_cmd[i]);
	}
}

void Backstepping::stopping(const ros::Time&)
{
	//TO DO
}

/* Check for the effort commanded */
Eigen::Matrix<double, 7, 1> Backstepping::saturateTorqueRate( 
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

void Backstepping::setCommandCB(const panda_controllers::desTrajEE::ConstPtr& msg)
{
 	ee_pos_cmd << msg->position.x, msg->position.y, msg->position.z;
	ee_vel_cmd << msg->velocity.x, msg->velocity.y, msg->velocity.z;
	ee_acc_cmd << msg->acceleration.x, msg->acceleration.y, msg->acceleration.z;
}

template <size_t N>
void Backstepping::fillMsg(boost::array<double, N>& msg_, const Eigen::VectorXd& data_) {
    
	int dim = data_.size();
    for (int i = 0; i < dim; i++) {
        msg_[i] = data_[i];
    }
}

}

PLUGINLIB_EXPORT_CLASS(panda_controllers::Backstepping, controller_interface::ControllerBase);
