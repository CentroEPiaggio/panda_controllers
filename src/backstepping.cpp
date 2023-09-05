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

	/* Assigning joint names */

	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != NJ) {
		ROS_ERROR("Backstepping: Error in parsing joints name!");
		return false;
	}
	for (size_t i = 0; i < NJ; ++i) {
		try {
			joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			ROS_ERROR_STREAM("Backstepping: Exception getting joint handles: " << ex.what());
			return false;
		}
	}

	/* Inizializing the Lambda and R and Kd gains */
	
	double gainKd;
	std::vector<double> gainR(NJ), gainLambda(6);
	Eigen::MatrixXd Rblock;
	
	if (!node_handle.getParam("gainLambda", gainLambda) ||
		!node_handle.getParam("gainR", gainR) ||
		!node_handle.getParam("gainKd", gainKd)  ||
		!node_handle.getParam("update_param", update_param_flag)) {
	
		ROS_ERROR("Backstepping: Could not get gain parameter for Lambda, R, Kd!");
		return false;
	}
	Lambda.setIdentity();
	for(int i=0;i<6;i++){
		Lambda(i,i) = gainLambda[i];
	}
	Kd = gainKd * Eigen::MatrixXd::Identity(NJ, NJ);
	R.setIdentity();
	Rinv.setZero();
	for (int i = 0; i<NJ; i++){
		Rblock = gainR[i]*R.block(i*PARAM, i*PARAM, PARAM, PARAM) ;
		if (update_param_flag && Rblock.determinant() != 0){
			Rinv.block(i*PARAM, i*PARAM, PARAM, PARAM) = Rblock.inverse();
		}
	}

	/* Assigning inertial parameters for initial guess of panda parameters to compute dynamics with regressor */

	for(int i=0; i<NJ; i++){
		double mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz;
		if (!node_handle.getParam("link"+std::to_string(i+1)+"_REG"+"/mass", mass) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_REG"+"/m_CoM_x", cmx) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_REG"+"/m_CoM_y", cmy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_REG"+"/m_CoM_z", cmz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_REG"+"/Ixx", xx) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_REG"+"/Ixy", xy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_REG"+"/Ixz", xz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_REG"+"/Iyy", yy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_REG"+"/Iyz", yz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_REG"+"/Izz", zz)){
			
			ROS_ERROR("Backstepping: Error in parsing inertial parameters!");
			return 1;
		}
		param.segment(PARAM*i, PARAM) << mass,cmx,cmy,cmz,xx,xy,xz,yy,yz,zz;
	}

	/* Assigning inertial parameters for initial guess of panda parameters to compute dynamics with standard dynamic equation */

	for(int i=0; i<NJ; i++){
		double mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz;
		if (!node_handle.getParam("link"+std::to_string(i+1)+"_DYN"+"/mass", mass) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_DYN"+"/CoM_x", cmx) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_DYN"+"/CoM_y", cmy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_DYN"+"/CoM_z", cmz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_DYN"+"/Ixx", xx) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_DYN"+"/Ixy", xy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_DYN"+"/Ixz", xz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_DYN"+"/Iyy", yy) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_DYN"+"/Iyz", yz) ||
			!node_handle.getParam("link"+std::to_string(i+1)+"_DYN"+"/Izz", zz)){
			
			ROS_ERROR("Backstepping: Error in parsing inertial parameters!");
			return 1;
		}
		param_dyn.segment(PARAM*i, PARAM) << mass,cmx,cmy,cmz,xx,xy,xz,yy,yz,zz;
	}

	/* Initialize joint (torque,velocity) limits */

	tau_limit << 87, 87, 87, 87, 12, 12, 12;
	q_max_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
	q_min_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
	q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

	/*Start command subscriber and advertise */

	this->sub_command_ = node_handle.subscribe<panda_controllers::desTrajEE> ("command", 1, &Backstepping::setCommandCB, this);   //it verify with the callback that the command has been received
	this->pub_err_ = node_handle.advertise<panda_controllers::log_adaptive_cartesian> ("logging", 1);
	this->pub_config_ = node_handle.advertise<panda_controllers::point> ("current_config", 1);
	
	/* Initialize regressor object */

	fastRegMat.init(NJ);
	fastRegMat.setInertialParam(param_dyn);
	
	return true;
}

void Backstepping::starting(const ros::Time& time)
{	
	/* Getting Robot State in order to get q_curr and dot_q_curr and jacobian of end-effector */
	
	franka::RobotState robot_state = state_handle_->getRobotState();
	Eigen::Affine3d T0EE(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.dq.data());

	/* Secure initialization command */
	ee_pos_cmd = T0EE.translation();
	ee_rot_cmd = T0EE.linear();

	ee_vel_cmd.setZero();
	ee_acc_cmd.setZero();

	ee_ang_vel_cmd.setZero();
	ee_ang_acc_cmd.setZero();

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
	Eigen::Matrix<double, 6, NJ> jacobian;
	Eigen::Affine3d T0EE;
	
	jacobian = Eigen::Map<Eigen::Matrix<double, 6, NJ>>(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());
	rosG = Eigen::Map<Eigen::Matrix<double, NJ, 1>> (model_handle_->getGravity().data());
	
	T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
	q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.dq.data());

	/* Update pseudo-inverse of jacobian and its derivative */

	fastRegMat.setArguments(q_curr,dot_q_curr);

	/* Compute pseudo-inverse of jacobian and its derivative */
	
	Eigen::MatrixXd mypJacEE = fastRegMat.getPinvJac_gen();
	Eigen::MatrixXd mydot_pJacEE = fastRegMat.getDotPinvJac_gen();

	/* Compute error translation */

	Eigen::Vector3d ee_position = T0EE.translation();
	Eigen::Vector3d ee_velocity = jacobian.topRows(3)*dot_q_curr;

	error.head(3) = ee_pos_cmd - ee_position;
	dot_error.head(3) = ee_vel_cmd - ee_velocity;

	/* Compute error orientation */

  	Eigen::MatrixXd ee_rot = T0EE.linear();
	Eigen::Vector3d ee_omega = jacobian.bottomRows(3)*dot_q_curr;

  	Eigen::MatrixXd Rs_tilde;
  	Eigen::Matrix<double,3,3> L;
  	Eigen::Matrix<double,3,3> dotL;
	Rs_tilde = ee_rot_cmd*ee_rot.transpose();
	L = createL(ee_rot_cmd, ee_rot);
	dotL = createDotL(ee_rot_cmd, ee_rot, ee_ang_vel_cmd, ee_omega);
	
	error.tail(3) = vect(Rs_tilde);
	dot_error.tail(3) = L.transpose()*ee_ang_vel_cmd-L*ee_omega;

	/* Compute reference */

	Eigen::VectorXd ee_vel_cmd_tot(6), ee_acc_cmd_tot(6);
	ee_vel_cmd_tot << ee_vel_cmd, L.transpose()*ee_ang_vel_cmd;
	ee_acc_cmd_tot << ee_acc_cmd, dotL.transpose()*ee_ang_vel_cmd + L.transpose()*ee_ang_acc_cmd;
	
	Eigen::VectorXd tmp_position = ee_vel_cmd_tot + Lambda * error;
	Eigen::VectorXd tmp_velocity = ee_acc_cmd_tot + Lambda * dot_error;
	Eigen::Matrix<double,6,6> tmp_conversion0,tmp_conversion1,tmp_conversion2;

	tmp_conversion0.setIdentity();
	tmp_conversion0.block(3, 3, 3, 3) = L;
	tmp_conversion1.setIdentity();
	tmp_conversion1.block(3, 3, 3, 3) = L.inverse();
	tmp_conversion2.setZero();
	tmp_conversion2.block(3, 3, 3, 3) = L.inverse() * dotL *L.inverse();

	dot_qr = mypJacEE*tmp_conversion1*tmp_position;
	ddot_qr = mypJacEE*tmp_conversion1*tmp_velocity + mypJacEE*tmp_conversion2*tmp_position +mydot_pJacEE*tmp_conversion1*tmp_position;

	s = dot_qr - dot_q_curr;
	
	/* Update and Compute Regressor */
	
	fastRegMat.setArguments(q_curr, dot_q_curr, dot_qr, ddot_qr);
	fastRegMat.setArguments(q_curr, dot_q_curr, param_dyn);
	
	Yr = fastRegMat.getReg_gen();

	/* tau_J_d is past tau_cmd saturated */

	tau_J_d = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.tau_J_d.data());

	/* Backstepping control law */

	dt = period.toSec();

	/* Update inertial parameters */
	
	if (update_param_flag){
		dot_param = Rinv*Yr.transpose()*s;
		param = param + dt*dot_param;
	}

/* ---------------------------------------------------------------------------- */
/* Eigen::MatrixXd myJacEE = fastRegMat.getJac_gen();
Eigen::MatrixXd myKin = fastRegMat.getKin_gen();
myM = fastRegMat.getMass_gen();
myC = fastRegMat.getCoriolis_gen();
myG = fastRegMat.getGravity_gen();

std::array<double, NJ*NJ> mass_array = model_handle_->getMass();
std::array<double, NJ> coriolis_array = model_handle_->getCoriolis();
rosM = Eigen::Map<Eigen::Matrix<double, NJ, NJ>>(mass_array.data());
rosC = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(coriolis_array.data());

std::cout<<"\n my jacobian \n"<<myJacEE<<std::endl;
std::cout<<"\n ros jacobian \n"<<jacobian<<std::endl;
std::cout<<"\n my kinematic \n"<<myKin<<std::endl;
std::cout<<"\n ros kinematic \n"<<T0EE.matrix()<<std::endl;
std::cout<<"\n my mass contribute \n"<<myM<<std::endl;
std::cout<<"\n ros mass contribute \n"<<rosM<<std::endl;
std::cout<<"\n my coriolis contribute \n"<<myC*dot_q_curr<<std::endl;
std::cout<<"\n ros coriolis contribute \n"<<rosC<<std::endl;
std::cout<<"\n my gravity contribute \n"<<myG<<std::endl;
std::cout<<"\n ros gravity contribute \n"<<rosG<<std::endl; */
/* ---------------------------------------------------------------------------- */

	/* Compute tau command */

	tau_cmd_dyn = myM*ddot_qr + myC*dot_qr + myG;
	tau_cmd_reg = Yr*param;

/* ---------------------------------------------------------------------------- */
//std::cout<<"\nfunzoina!:\n"<<tau_cmd_dyn - tau_cmd_reg<<"\n========\n";
/* ---------------------------------------------------------------------------- */

	//tau_cmd = tau_cmd_reg + Kd*s + jacobian.transpose()*error-rosG;
	tau_cmd = tau_cmd_reg + Kd*s + jacobian.transpose()*tmp_conversion0.transpose()*error-rosG;
	//tau_cmd.setZero();
/* 
std::cout<<"\njacobian\n"<<jacobian.transpose()<<"\n";
std::cout<<"\njacobian.topRows(space_).transpose():\n"<<jacobian.topRows(space_).transpose()<<"\n"; */

	/* Saturate the derivate of tau_cmd */
	
	tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
//std::cout<<"\ntau_cmd saturated \n"<<tau_cmd<<std::endl;
 	/* Publish messages */

	time_now = ros::Time::now();
	
	msg_log.header.stamp = time_now;
	fillMsg(msg_log.error_pos_EE, error);
	fillMsg(msg_log.dot_error_pos_EE, dot_error);
	fillMsg(msg_log.dot_qr, dot_qr);
	fillMsg(msg_log.ddot_qr, ddot_qr);
	fillMsg(msg_log.s, s);
	fillMsgLink(msg_log.link1, param.segment(0, PARAM));
	fillMsgLink(msg_log.link2, param.segment(10, PARAM));
	fillMsgLink(msg_log.link3, param.segment(20, PARAM));
	fillMsgLink(msg_log.link4, param.segment(30, PARAM));
	fillMsgLink(msg_log.link5, param.segment(40, PARAM));
	fillMsgLink(msg_log.link6, param.segment(50, PARAM));
	fillMsgLink(msg_log.link7, param.segment(60, PARAM));
	fillMsg(msg_log.tau_cmd, tau_cmd);

	msg_config.header.stamp  = time_now;
	msg_config.xyz.x = T0EE.translation()(0);
	msg_config.xyz.y = T0EE.translation()(1);
	msg_config.xyz.z = T0EE.translation()(2);

	this->pub_err_.publish(msg_log);
	this->pub_config_.publish(msg_config);
	
	/* Set the command for each joint */

	for (size_t i = 0; i < NJ; i++) {
		joint_handles_[i].setCommand(tau_cmd[i]);
	}
}

void Backstepping::stopping(const ros::Time&)
{
	//TO DO
}

/* Check for the effort commanded */
Eigen::Matrix<double, NJ, 1> Backstepping::saturateTorqueRate( 
	const Eigen::Matrix<double, NJ, 1>& tau_d_calculated, const Eigen::Matrix<double, NJ, 1>& tau_J_d){
	
	Eigen::Matrix<double, NJ, 1> tau_d_saturated {};
	for (size_t i = 0; i < NJ; i++) {

		double difference = tau_d_calculated[i] - tau_J_d[i];
		tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);

/* 		if(fabs(tau_d_saturated[i])>tau_limit[i]){
			tau_d_saturated[i] = (tau_d_saturated[i]/fabs(tau_d_saturated[i]))*tau_limit[i];
		} */
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

void Backstepping::fillMsgLink(panda_controllers::link_params &msg_, const Eigen::VectorXd& data_) {
    
    msg_.mass = data_[0];
	msg_.m_CoM_x = data_[1];
	msg_.m_CoM_y = data_[2];
	msg_.m_CoM_z = data_[3];
	msg_.Ixx = data_[4];
	msg_.Ixy = data_[5];
	msg_.Ixz = data_[6];
	msg_.Iyy = data_[7];
	msg_.Iyz = data_[8];
	msg_.Izz = data_[9];
}

}

PLUGINLIB_EXPORT_CLASS(panda_controllers::Backstepping, controller_interface::ControllerBase);
