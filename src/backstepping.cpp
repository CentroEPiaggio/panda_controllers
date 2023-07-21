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
	
	double gainLambda, gainR, gainKd;
	if (!node_handle.getParam("gainLambda", gainLambda) ||
		!node_handle.getParam("gainR", gainR) ||
		!node_handle.getParam("gainKd", gainKd)) {
		ROS_ERROR("Backstepping: Could not get gain parameter for Lambda, R, Kd!");
		return false;
	}

	Lambda = gainLambda * Eigen::Matrix3d::Identity();
	R = gainR * Eigen::MatrixXd::Identity(70, 70);
	Kd = gainKd * Eigen::MatrixXd::Identity(7, 7);

	/* Assigning the time */
   
	if (!node_handle.getParam("dt", dt)) {
		ROS_ERROR("Backstepping: Could not get parameter dt!");
		return false;
	}

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
	
	/* Assigning inertial parameters */

	std::vector<double> link_1_arr, link_2_arr, link_3_arr, link_4_arr, link_5_arr, link_6_arr, link_7_arr, hand_arr;
	if (!node_handle.getParam("link_1", link_1_arr) || link_1_arr.size() != 10 ||
		!node_handle.getParam("link_2", link_2_arr) || link_2_arr.size() != 10 ||
		!node_handle.getParam("link_3", link_3_arr) || link_3_arr.size() != 10 ||
		!node_handle.getParam("link_4", link_4_arr) || link_4_arr.size() != 10 ||
		!node_handle.getParam("link_5", link_5_arr) || link_5_arr.size() != 10 ||
		!node_handle.getParam("link_6", link_6_arr) || link_6_arr.size() != 10 ||
		!node_handle.getParam("link_7", link_7_arr) || link_7_arr.size() != 10 ||
		!node_handle.getParam("end_effector", hand_arr) || hand_arr.size() != 10 ) {
		ROS_ERROR("Backstepping: Error in parsing inertial parameters!");
		return false;
	}

	Eigen::VectorXd link_1 = Eigen::Map<Eigen::VectorXd>(link_1_arr.data(), 10, 1);
	Eigen::VectorXd link_2 = Eigen::Map<Eigen::VectorXd>(link_2_arr.data(), 10, 1);
	Eigen::VectorXd link_3 = Eigen::Map<Eigen::VectorXd>(link_3_arr.data(), 10, 1);
	Eigen::VectorXd link_4 = Eigen::Map<Eigen::VectorXd>(link_4_arr.data(), 10, 1);
	Eigen::VectorXd link_5 = Eigen::Map<Eigen::VectorXd>(link_5_arr.data(), 10, 1);
	Eigen::VectorXd link_6 = Eigen::Map<Eigen::VectorXd>(link_6_arr.data(), 10, 1);
	Eigen::VectorXd link_7 = Eigen::Map<Eigen::VectorXd>(link_7_arr.data(), 10, 1);
	Eigen::VectorXd hand = Eigen::Map<Eigen::VectorXd>(hand_arr.data(), 10, 1);

	param << link_1, link_2, link_3, link_4, link_5, link_6, link_7;

	/* Initialize joint (torque,velocity) limits */

	tau_limit << 87, 87, 87, 87, 12, 12, 12;
	q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

	/*Start command subscriber and advertise */

	this->sub_command_ = node_handle.subscribe<panda_controllers::desTrajEE> ("command", 1, &Backstepping::setCommandCB, this);   //it verify with the callback that the command has been received
	this->pub_err_ = node_handle.advertise<panda_controllers::log_backstepping> ("logging", 1);
	
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

	double mass_ee = robot_state.m_ee;
	Eigen::Matrix<double,3,3> inertial_ee = Eigen::Map<Eigen::Matrix<double, 3, 3>>(robot_state.I_ee.data());

/* 	std::cout<<"\nmass\n"<<mass_ee<<std::endl;
	std::cout<<"\ninertia\n"<<inertial_ee<<std::endl;
 */
	/* Secure initialization command */

	ee_pos_cmd = T0EE.translation();
	//ee_pos_cmd << 0,0.307,0.487;
	//ee_pos_cmd << 0.6,0,0.8;
	ee_vel_cmd = Eigen::Vector3d::Zero();
	ee_acc_cmd = Eigen::Vector3d::Zero();

	/* Compute error */

	error = Eigen::Vector3d::Zero();
	dot_error = Eigen::Vector3d::Zero();
	
	/* Compute reference (Position Control) */
	
	dot_qr = Eigen::VectorXd::Zero(7,1);
	ddot_qr = Eigen::VectorXd::Zero(7,1);
	
	/* Update regressor */

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

	/* tau_J_d is the desired link-side joint torque sensor signals without gravity */

	tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

	/* Backstepping control law */

	Eigen::Matrix<double,70,1> param_new;
	Eigen::Matrix<double,70,1> dot_param;
	double delta_t;

	/* Update inertial parameters */
	
	dot_param = R.inverse()*Yr.transpose()*s;
	delta_t = (double)period.sec+(double)period.nsec*1e-9;
 	
	param_new = param + delta_t*dot_param;
	param = param_new;
 
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

	this->pub_err_.publish(msg_log);

	/* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */
	
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
