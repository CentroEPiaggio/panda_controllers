//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/backstepping.h> //library of the Backstepping 

namespace panda_controllers{

bool Backstepping::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{ 
	this->cvc_nh = node_handle;
	
	std::string arm_id; //checking up the arm id of the robot
	if (!node_handle.getParam("arm_id", arm_id)) {
		ROS_ERROR("Backstepping: Could not get parameter arm_id!");
		return false;
	}

	/* Inizializing the Kp and Kv gains */

	double kp1, kp2, kp3, kv1, kv2, kv3;
	
	if (!node_handle.getParam("kp1", kp1) || !node_handle.getParam("kp2", kp2) || !node_handle.getParam("kp3", kp3) 
		|| !node_handle.getParam("kv1", kv1) || !node_handle.getParam("kv2", kv2) || !node_handle.getParam("kv3", kv3)) {
		ROS_ERROR("Backstepping: Could not get parameter kpi or kv!");
		return false;
	}

	Kp = Eigen::MatrixXd::Identity(7, 7);
	Kp(0,0) = kp1; Kp(1,1) = kp1; Kp(2,2) = kp1; Kp(3,3) = kp1; Kp(4,4) = kp2; Kp(5,5) = kp2; Kp(6,6) = kp3;
	
	Kv = Eigen::MatrixXd::Identity(7, 7);
	Kv(0,0) = kv1; Kv(1,1) = kv1; Kv(2,2) = kv1; Kv(3,3) = kv1; Kv(4,4) = kv2; Kv(5,5) = kv2; Kv(6,6) = kv3;
	
	/* Assigning the time */
   
	if (!node_handle.getParam("dt", dt)) {
		ROS_ERROR("Backstepping: Could not get parameter dt!");
		return false;
	}

	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		ROS_ERROR("Backstepping: Error in parsing joints name!");
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

	for (size_t i = 0; i < 7; ++i) {
		try {
			joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			ROS_ERROR_STREAM("Backstepping: Exception getting joint handles: " << ex.what());
			return false;
		}
	}
	
	/* Initialize joint (torque,velocity) limits */

	tau_limit << 87, 87, 87, 87, 12, 12, 12;
	q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

	/*Start command subscriber */

	this->sub_command_ = node_handle.subscribe<panda_controllers::desTrajEE> ("command", 1, &Backstepping::setCommandCB, this);   //it verify with the callback that the command has been received
	this->pub_err_ = node_handle.advertise<panda_controllers::log_backstepping> ("logging", 1);
	
	/* Initialize regressor object */

	const int nj = 7;
	/* const std::string jTypes = "RRRRRRR";
	Eigen::MatrixXd DHTable(nj,4);

	// DHTable obatain in URDF
	DHTable << 	0,		-M_PI_2,	0.3330, 0,
                0,      M_PI_2,  	0,      0,
                0.0825, M_PI_2,  	0.3160, 0,
               -0.0825, -M_PI_2,	0,      0,
                0,      M_PI_2,  	0.384,  0,
                0.088,  M_PI_2,  	0,      0,
                0,      0,         	0.107,  0;
 
	regrob::FrameOffset base_to_L0({0,0,0},{0,0,0},{0,0,-9.8});
	regrob::FrameOffset L0_to_EE({-M_PI_4,0,0},{0,0,0.1034},{0,0,-9.8});
	regressor.init(nj, DHTable, jTypes, base_to_L0, L0_to_EE); */

	fastRegMat.init(nj);

	/* Initialize inertial parameters */

	param = importCSV("/home/yurs/franka_ws/src/panda_controllers/src/start_parameters.csv");
	//std::cout<<param<<std::endl;

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
	Eigen::Matrix<double,6,1> inertial_ee = Eigen::Map<Eigen::Matrix<double, 6, 1>>(robot_state.I_ee.data());

	std::cout<<"mass\n"<<mass_ee<<std::endl;
	std::cout<<"inertia\n"<<inertial_ee<<std::endl;
	

	std::cout<<q_curr<<std::endl;

	/* Secure initialization command */

	ee_pos_cmd = T0EE.translation();
	//ee_pos_cmd << 0,0.307,0.487;
	//ee_pos_cmd << 0.6,0,0.8;
	ee_vel_cmd = Eigen::Vector3d::Zero();
	ee_acc_cmd = Eigen::Vector3d::Zero();

	/* Compute error */

	Eigen::Vector3d error = Eigen::Vector3d::Zero();
	Eigen::Vector3d dot_error = Eigen::Vector3d::Zero();
	
	/* Compute reference (Position Control) */
	
	Eigen::Matrix<double,7,1> dot_qr = Eigen::VectorXd::Zero(7,1);
	Eigen::Matrix<double,7,1> ddot_qr = Eigen::VectorXd::Zero(7,1);
	
	/* Update regressor */
/* 
	regressor.setArguments(q_curr, dot_q_curr, dot_qr, ddot_qr);*/
    fastRegMat.setArguments(q_curr, dot_q_curr, dot_qr, ddot_qr);

	Kp_apix = Kp;
	Kv_apix = Kv; 
}

void Backstepping::update(const ros::Time&, const ros::Duration& period)
{

	/* Getting Robot State in order to get q_curr and dot_q_curr and jacobian of end-effector */
	
	franka::RobotState robot_state = state_handle_->getRobotState();
	Eigen::Affine3d T0EE(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
	std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
	Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());

	std::array<double, 7> gravity_array = model_handle_->getGravity();
	Eigen::Map<Eigen::Matrix<double, 7, 1> > G(gravity_array.data());
	
	/* Update Jacobian Function */
	/* 
	regressor.setArguments(q_curr,dot_q_curr); */
	fastRegMat.setArguments(q_curr,dot_q_curr);

	/* Compute pseudo-inverse of jacobian and its derivative */

	Eigen::MatrixXd mypJacEE = fastRegMat.getPinvJac_gen();
	Eigen::MatrixXd mydot_pJacEE = fastRegMat.getDotPinvJac_gen();
/* 
	mypJacEE = regressor.pinvJacobian();
	mydot_pJacEE = regressor.dotPinvJacobian(); */

	/* Compute error */

	Eigen::Vector3d ee_position = T0EE.translation();
	Eigen::Vector3d ee_velocity = jacobian.topRows(3)*dot_q_curr;

	Eigen::Vector3d error = ee_pos_cmd - ee_position;
	Eigen::Vector3d dot_error = ee_vel_cmd - ee_velocity;

	/* Compute reference (Position Control) */

	Eigen::Matrix3d Lambda = Eigen::Matrix3d::Identity()*1;	// da mettere su init e inizializzare con rosparam
	
	Eigen::Vector3d tmp_position = ee_vel_cmd + Lambda * error;
	Eigen::Vector3d tmp_velocity = ee_acc_cmd + Lambda * dot_error;
	
	Eigen::Matrix<double,7,1> dot_qr = mypJacEE * tmp_position;
	Eigen::Matrix<double,7,1> ddot_qr = mypJacEE * tmp_velocity + mydot_pJacEE * tmp_position;
	Eigen::Matrix<double,7,1> s = dot_qr - dot_q_curr;
	
	/* Update Regressor Function */
/* 
	regressor.setArguments(q_curr, dot_q_curr, dot_qr, ddot_qr); */

	fastRegMat.setArguments(q_curr, dot_q_curr, dot_qr, ddot_qr);
	Yr = fastRegMat.getReg_gen();

	/* Gain Matrix */ 
	Kp_apix = Kp;
	Kv_apix = Kv;

	/* tau_J_d is the desired link-side joint torque sensor signals without gravity */

	tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

	/* Backstepping control law */

	Eigen::Matrix<double,7,7> Kd = Eigen::Matrix<double,7,7>::Identity();	// da mettere su init e inizializzare con rosparam
	Eigen::Matrix<double,70,70> R = Eigen::Matrix<double,70,70>::Identity();	// da mettere su init e inizializzare con rosparam
	Eigen::Matrix<double,70,1> param_new;
	Eigen::Matrix<double,70,1> dot_param;
	double delta_t;
/* 
	Eigen::MatrixXd Yr = regressor.allColumns(); */

	//Yr = regressor.getRegressor_gen();
	/* Update inertial parameters */
	
	dot_param = R.inverse()*Yr.transpose()*s;
	delta_t = (double)period.sec+(double)period.nsec*1e-9;
 	
	param_new = param + delta_t*dot_param;
	param = param_new;
 
	/* Update tau and parameters */
	
	tau_cmd = Yr*param + Kd*s + jacobian.topRows(3).transpose()*error-G;
	
 	// Publish tracking errors as joint states
	panda_controllers::log_backstepping msg_log;

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

	/* for (int i = 0; i < 7; ++i) {
		msg_log.joint_angles[i] = q_curr[i];
	} */
	
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

Eigen::Matrix<double, 70, 1> Backstepping::importCSV(const std::string& filename) {

    std::vector<double> data;
    std::string line;
    std::string absolutePath = boost::filesystem::absolute(filename).string();

    std::cout << "Percorso assoluto del file: " << absolutePath << std::endl;
    std::ifstream file(filename);
    
	Eigen::Matrix<double, 70, 1> param_;

	if (!file.is_open()) {
        throw std::runtime_error("Impossibile aprire il file " + filename);
    }

    while (std::getline(file, line)) {

        std::stringstream ss(line);
        std::string cell;

        std::getline(ss, cell, ',');
    	data.push_back(std::stod(cell));
    }

    file.close();

	for (int i=0; i<70; i++)
		param_[i] = data[i];

    return param_;
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
