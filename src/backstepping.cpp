//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <backstepping.h> //library of the Backstepping
#include <ros/package.h>

using namespace std;
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

	if (!node_handle.getParam("logging", logging)) {
		ROS_ERROR("Backstepping: Could not get parameter logging, set to 0!");
		logging = false;
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

	/* Assigning inertial parameters for initial guess of panda parameters to compute dynamics with regressor */
	// for(int i=0; i<NJ; i++){
	// 	double mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz;
	// 	if (!node_handle.getParam("link"+std::to_string(i+1)+"/mass", mass) ||
	// 		!node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_x", cmx) ||
	// 		!node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_y", cmy) ||
	// 		!node_handle.getParam("link"+std::to_string(i+1)+"/m_CoM_z", cmz) ||
	// 		!node_handle.getParam("link"+std::to_string(i+1)+"/Ixx", xx) ||
	// 		!node_handle.getParam("link"+std::to_string(i+1)+"/Ixy", xy) ||
	// 		!node_handle.getParam("link"+std::to_string(i+1)+"/Ixz", xz) ||
	// 		!node_handle.getParam("link"+std::to_string(i+1)+"/Iyy", yy) ||
	// 		!node_handle.getParam("link"+std::to_string(i+1)+"/Iyz", yz) ||
	// 		!node_handle.getParam("link"+std::to_string(i+1)+"/Izz", zz)){
			
	// 		ROS_ERROR("Backstepping: Error in parsing inertial parameters!");
	// 		return 1;
	// 	}
	// 	param_REG.segment(PARAM*i, PARAM) << mass,cmx,cmy,cmz,xx,xy,xz,yy,yz,zz;
	// }


	// - thunder init - //
	// get absolute path to franka_conf.yaml file
	std::string package_path = ros::package::getPath("panda_controllers");
	std::string path_conf = package_path + "/config/thunder/franka.yaml";
	std::string path_par_REG = package_path + "/config/thunder/franka_par_REG_pW.yaml";
	frankaRobot.load_conf(path_conf);
	frankaRobot.load_par_REG(path_par_REG);
	param_REG = frankaRobot.get_par_REG();
	param_init = param_REG;
	// ee_tr.setZero();

	/* Inizializing the Lambda and R and Kd gains */
	
	std::vector<double> gainRlinks(NJ), gainRparam(4), gainLambda(6), gainKd(7);
	Eigen::Matrix<double,PARAM,PARAM> Rlink;

	if (!node_handle.getParam("gainLambda", gainLambda) ||
		!node_handle.getParam("gainRlinks", gainRlinks) ||
		!node_handle.getParam("gainRparam", gainRparam) ||
		!node_handle.getParam("gainKd", gainKd)  ||
		!node_handle.getParam("adaptive_kin", update_kin_flag)||
		!node_handle.getParam("adaptive_dyn", update_dyn_flag)||
		!node_handle.getParam("upper_bound_s", UB_s_flag)) {
	
		ROS_ERROR("Backstepping: Could not get gain parameter for Lambda, R, Kd!");
		return false;
	}
	Lambda.setIdentity();
	for(int i=0;i<6;i++){
		Lambda(i,i) = gainLambda[i];
	}
	Kd.setIdentity();
	for(int i=0;i<NJ;i++){
		Kd(i,i) = gainKd[i];
	}
	
	Rlink.setZero();
	Rlink(0,0) = gainRparam[0];
	Rlink(1,1) = gainRparam[1];
	Rlink(2,2) = Rlink(1,1);
	Rlink(3,3) = Rlink(1,1);
	Rlink(4,4) = gainRparam[2];
	Rlink(5,5) = gainRparam[3];
	Rlink(6,6) = Rlink(5,5);
	Rlink(7,7) = Rlink(4,4);
	Rlink(8,8) = Rlink(5,5);;
	Rlink(9,9) = Rlink(4,4);

	Rinv.setZero();
	for (int i = 0; i<NJ; i++){	
		Rinv.block(i*PARAM, i*PARAM, PARAM, PARAM) = gainRlinks[i]*Rlink;;
	}

	/* Initialize joint (torque,velocity) limits */

	tau_limit << 87, 87, 87, 87, 12, 12, 12;
	q_max_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
	q_min_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
	q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

	/*Start command subscriber and advertise */

	this->sub_command_ = node_handle.subscribe<panda_controllers::desTrajEE> ("command_cartesian", 1, &Backstepping::setCommandCB, this);
	this->sub_flag_update_ = node_handle.subscribe<panda_controllers::flag> ("adaptiveFlag", 1, &Backstepping::setFlagUpdate, this);
	// this->sub_dyn_update_ = node_handle.subscribe<panda_controllers::flag>", 1, &Backstepping::setDynUpdate, this);
	this->pub_log = node_handle.advertise<panda_controllers::log_adaptive_cartesian> ("logging", 1);
	this->pub_config_ = node_handle.advertise<panda_controllers::point> ("current_config", 1);

	// set end effector
	// Creazione di un client di servizio per SetEEFrame
  	ros::ServiceClient client = node_handle.serviceClient<franka_msgs::SetEEFrame>("/franka_control/set_EE_frame");

	franka_msgs::SetEEFrame srv;

	// // transformation matrix (column_major)
	// srv.request.NE_T_EE[0] = 1.0;
	// srv.request.NE_T_EE[1] = 0.0;
	// srv.request.NE_T_EE[2] = 0.0;
	// srv.request.NE_T_EE[3] = 0.0;

	// srv.request.NE_T_EE[4] = 0.0;
	// srv.request.NE_T_EE[5] = 1.0;
	// srv.request.NE_T_EE[6] = 0.0;
	// srv.request.NE_T_EE[7] = 0.0;

	// srv.request.NE_T_EE[8] = 0.0;
	// srv.request.NE_T_EE[9] = 0.0;
	// srv.request.NE_T_EE[10] = 1.0;
	// srv.request.NE_T_EE[11] = 0.0;

	// srv.request.NE_T_EE[12] = 0.0;  // translation X
	// srv.request.NE_T_EE[13] = 0.0;  // translation Y
	// srv.request.NE_T_EE[14] = 0.0;  // translation Z
	// srv.request.NE_T_EE[15] = 1.0;

	// // Invocazione del servizio
	// if (client.call(srv)) {
	// 	if (srv.response.success) {
	// 	ROS_INFO("End Effector frame successfully set.");
	// 	} else {
	// 	ROS_ERROR("Failed to set End Effector frame.");
	// 	}
	// } else {
	// 	ROS_ERROR("Service call failed.");
	// }

	// set ee in thunder
	// Eigen::Matrix<double,3,1> ee_tr;
	// ee_tr << 0.2, 0.2, 0.2;
	// frankaRobot.set_Ln2EE(ee_tr);
	
	return true;
}

void Backstepping::starting(const ros::Time& time)
{	
	/* Getting Robot State in order to get q_curr and dot_q_curr and jacobian of end-effector */
	
	robot_state = state_handle_->getRobotState();
	Eigen::Matrix4d T0EE_real = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());

	q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.dq.data());

	frankaRobot.set_q(q_curr);
	frankaRobot.set_dq(dot_q_curr);
	T0EE = frankaRobot.get_T_0_ee();
	ee_tr = frankaRobot.get_Ln2EE();
	cout << "ee_tr: " << ee_tr << endl;
	cout << "T0EE_real: \n" << T0EE_real << endl<<endl;
	cout << "T0EE_thunder: \n" << T0EE << endl<<endl;

	Eigen::Matrix<double,6,NJ> Jee_real = Eigen::Map<Eigen::Matrix<double, 6, NJ>> (model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());
	Jee = frankaRobot.get_J_ee();
	cout << "Jee_real: \n" << Jee_real << endl<<endl;
	cout << "Jee_thunder: \n" << Jee << endl<<endl;

	Eigen::Matrix<double,NJ,NJ> M_real = Eigen::Map<Eigen::Matrix<double, NJ, NJ>> (model_handle_->getMass().data());
	Eigen::Matrix<double,NJ,NJ> M = frankaRobot.get_M();
	cout << "M_real: \n" << M_real << endl<<endl;
	cout << "M_thunder: \n" << M << endl<<endl;

	Eigen::Matrix<double,NJ,1> Cdq_real = Eigen::Map<Eigen::Matrix<double, NJ, 1>> (model_handle_->getCoriolis().data());
	Eigen::Matrix<double,NJ,NJ> C = frankaRobot.get_C();
	cout << "Cdq_real: \n" << Cdq_real << endl<<endl;
	cout << "Cdq_thunder: \n" << C*dot_q_curr << endl<<endl;

	Eigen::Matrix<double,NJ,1> G_real = Eigen::Map<Eigen::Matrix<double, NJ, 1>> (model_handle_->getGravity().data());
	Eigen::Matrix<double,NJ,1> G = frankaRobot.get_G();
	cout << "G_real: \n" << G_real << endl<<endl;
	cout << "G_thunder: \n" << G << endl<<endl;

	/* Secure initialization command */
	ee_pos_cmd = T0EE_real.block<3,1>(0,3);
	ee_rot_cmd = T0EE_real.block<3,3>(0,0);
	// ee_pos_cmd = T0EE.block<3,1>(0,3);
	// ee_rot_cmd = T0EE.block<3,3>(0,0);
	
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
}

void Backstepping::update(const ros::Time&, const ros::Duration& period)
{
	bool saturate_s_flag;
	/* Getting Robot State in order to get q_curr and dot_q_curr and jacobian of end-effector */
	
	robot_state = state_handle_->getRobotState();
	
	franka_G = Eigen::Map<Eigen::Matrix<double, NJ, 1>> (model_handle_->getGravity().data());
	// Jee = Eigen::Map<Eigen::Matrix<double, 6, NJ>>(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());
	// T0EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
	q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.q.data());
	dot_q_curr = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.dq.data());
	/* tau_J_d is past tau_cmd saturated */
	tau_J_d = Eigen::Map<Eigen::Matrix<double, NJ, 1>>(robot_state.tau_J_d.data());
	frankaRobot.set_q(q_curr);
	frankaRobot.set_dq(dot_q_curr);
	Jee = frankaRobot.get_J_ee();
	T0EE = frankaRobot.get_T_0_ee();
	Eigen::Matrix4d T0EE_real = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
	Eigen::Matrix<double,6,NJ> Jee_real = Eigen::Map<Eigen::Matrix<double, 6, NJ>> (model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());
	/* Compute pseudo-inverse of jacobian and its derivative */
	J_pinv = frankaRobot.get_J_ee_pinv();
	J_dot = frankaRobot.get_J_ee_dot();
	Eigen::Vector3d ee_pos, ee_vel, ee_pos_real, ee_vel_real;
	Eigen::Vector3d ee_omega, ee_omega_real;
	Eigen::VectorXd ee_vel_cmd_tot(6), ee_acc_cmd_tot(6);
	Eigen::VectorXd tmp_position(6), tmp_velocity(6);
	
  	Eigen::Matrix<double,3,3> ee_rot;
	Eigen::Matrix<double,3,3> ee_rot_real;
  	Eigen::Matrix<double,3,3> Rs_tilde;
	Eigen::Matrix<double,3,3> Rs_tilde_real;
  	Eigen::Matrix<double,3,3> L, dotL;
	Eigen::Matrix<double,6,1> error_real;
	Eigen::Matrix<double,6,1> dot_error_real;

	Eigen::Matrix<double,6,6> tmp_conversion0, tmp_conversion1, tmp_conversion2;

	// - Forward kinematics - //
	ee_pos = T0EE.block<3,1>(0,3);
	ee_pos_real = T0EE_real.block<3,1>(0,3);
	ee_vel = Jee.topRows(3)*dot_q_curr;
	ee_vel_real = Jee_real.topRows(3)*dot_q_curr;
	ee_rot = T0EE.block<3,3>(0,0);
	ee_rot_real = T0EE_real.block<3,3>(0,0);
	ee_omega = Jee.bottomRows(3)*dot_q_curr;
	ee_omega_real = Jee_real.bottomRows(3)*dot_q_curr;

	// - Error computations - //
	Rs_tilde = ee_rot_cmd*ee_rot.transpose();
	Rs_tilde_real = ee_rot_cmd * ee_rot_real.transpose();
	L = createL(ee_rot_cmd, ee_rot);
	dotL = createDotL(ee_rot_cmd, ee_rot, ee_ang_vel_cmd, ee_omega);

	error.head(3) = ee_pos_cmd - ee_pos;
	error_real.head(3) = ee_pos_cmd - ee_pos_real;
	dot_error.head(3) = ee_vel_cmd - ee_vel;
	dot_error_real.head(3) = ee_vel_cmd - ee_vel_real;
	
	error.tail(3) = vect(Rs_tilde);
	error_real.tail(3) = vect(Rs_tilde_real);
	dot_error.tail(3) = L.transpose()*ee_ang_vel_cmd-L*ee_omega;
	dot_error_real.tail(3) = L.transpose()*ee_ang_vel_cmd-L*ee_omega_real;
	/* Compute reference */
	ee_vel_cmd_tot << ee_vel_cmd, L.transpose()*ee_ang_vel_cmd;
	ee_acc_cmd_tot << ee_acc_cmd, dotL.transpose()*ee_ang_vel_cmd + L.transpose()*ee_ang_acc_cmd;

	// - kinematic error estimate - //
	Eigen::VectorXd e_pos_kin = ee_pos_real - ee_pos;
	Eigen::Matrix<double,3,3> Rs_tilde_kin = ee_rot_real * ee_rot.transpose();
	Eigen::MatrixXd e_rot_kin = vect(Rs_tilde_kin);
	Eigen::Matrix<double,6,1> e_kin;
	e_kin.head(3) = e_pos_kin;
	e_kin.tail(3) = e_rot_kin;
	Eigen::Matrix<double,6,1> dot_e_kin;
	dot_e_kin.head(3) = ee_vel_real - ee_vel;
	dot_e_kin.tail(3) = ee_omega_real - ee_omega;
	
	// conversions
	tmp_position = ee_vel_cmd_tot + Lambda * error;
	tmp_velocity = ee_acc_cmd_tot + Lambda * dot_error;
	
	tmp_conversion0.setIdentity();
	tmp_conversion0.block(3, 3, 3, 3) = L;
	tmp_conversion1.setIdentity();
	tmp_conversion1.block(3, 3, 3, 3) = L.inverse();
	tmp_conversion2.setZero();
	tmp_conversion2.block(3, 3, 3, 3) = -L.inverse() * dotL *L.inverse();
	dot_qr = J_pinv*tmp_conversion1*tmp_position;
	ddot_qr = J_pinv * (tmp_conversion1*tmp_velocity + tmp_conversion2*tmp_position + J_dot*dot_qr);

	s = dot_qr - dot_q_curr;
/* 	for(int i=0;i<NJ;i++){
		if (s(i,1)<=tol_s) s(i,1) = 0;
	} */
	
	/* Update and Compute Regressor */
	frankaRobot.set_dqr(dot_qr);
	frankaRobot.set_ddqr(ddot_qr);
	Yr = frankaRobot.get_Yr();
	/* Backstepping control law */
	dt = period.toSec();

	/* Update inertial parameters */
	Eigen::Matrix<double,7,1> s_temp = s;
	for(int i=0;i<NJ;i++){
		if (std::fabs(s_temp(i,1)) < tol_s) s_temp(i,1) = 0.0;
	}

	// /*skip update if s>UB_s*/
	// if (UB_s_flag){
	// 	int count_UB_s = 0;
	// 	for(int i=0;i<NJ;i++){
	// 		if (std::fabs(s_temp(i,1)) < tol_s){
	// 			s_temp(i,1) = 0.0;
	// 		}
	// 		if (std::fabs(s(i,1))>UB_s){
	// 			saturate_s_flag = true;
	// 		}else{
	// 			count_UB_s++;
	// 		}
	// 		if (count_UB_s == NJ) saturate_s_flag = false;
	// 	}
	// }

	// --- kinematics adaptive law --- //
	if (update_kin_flag){
		// - J^T method - //
		// Eigen::VectorXd w = tmp_conversion0.transpose()*error_real;
		// // Eigen::VectorXd w = tmp_conversion0.transpose()*e_kin;
		// frankaRobot.set_w(w);
		// Eigen::MatrixXd Yk = frankaRobot.get_reg_JTw();
		// Eigen::VectorXd dot_kin_par = -Yk.transpose() * dot_qr;
		// - J method - //
		// Eigen::MatrixXd Yk = frankaRobot.get_reg_Jdq();
		// Eigen::VectorXd dot_kin_par = - Yk.transpose() * tmp_conversion0.transpose() * error_real;
		// - Indirect method - //
		Eigen::MatrixXd Yk = frankaRobot.get_reg_Jdq();
		Eigen::VectorXd dot_kin_par = Yk.transpose() * dot_e_kin;

		ee_tr += dt * dot_kin_par;
		frankaRobot.set_Ln2EE(ee_tr);
		// cout << "Yk: \n" << Yk << endl;
	}
	// --- end --- //
	
	// --- dynamics adaptive law --- //
	if (update_dyn_flag){// && !saturate_s_flag){
		dot_param = Rinv*Yr.transpose()*s;//_temp;
/* 		std::cout<<"\n =================== \n s_temp:\n"<<s_temp<<"\n ------------------- \n";
		std::cout<<"\n dot_param: \n"<<dot_param<<"\n =================== \n"; */
		param_REG += dt*dot_param;
	}
	// --- end --- //

	// cout << "ee_tr: " << ee_tr.transpose() << endl;
	// cout << "e_pos: " << error.head(3).transpose()  << endl;
	// cout << "e_kin: " << e_kin.transpose()  << endl;
	// cout << "error: " << error_real.transpose() << endl;
	// Eigen::Quaterniond ee_rot_quat(ee_rot_real);
	// cout << "ee_rot_w: " << ee_rot_quat.w() << ", ee_rot_vec: " << ee_rot_quat.vec().transpose() << endl;

	/* Compute tau command */
	tau_tilde = Yr*(param_init-param_REG);
	tau_cmd = Yr*param_REG + Kd*s + Jee.transpose()*tmp_conversion0.transpose()*error;
	//tau_cmd.setZero(); // gravity compensation check (spoiler: it is not perfect)
	
	/* Saturate the derivate of tau_cmd */
	tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d + franka_G);
	
	/* Set the command for each joint */

	for (size_t i = 0; i < NJ; i++) {
		joint_handles_[i].setCommand(tau_cmd[i] - franka_G[i]);
	}

	/* Publish messages */
	time_now = ros::Time::now();

	if (logging){
	msg_log.header.stamp = time_now;
	
	fillMsg(msg_log.error_pos_EE, error_real);
	fillMsg(msg_log.dot_error_pos_EE, dot_error_real);
	fillMsg(msg_log.dot_qr, dot_qr);
	fillMsg(msg_log.ddot_qr, ddot_qr);
	fillMsg(msg_log.J, Jee);
	fillMsg(msg_log.J_dot, J_dot);
	fillMsg(msg_log.J_pinv, J_pinv);
	fillMsg(msg_log.L, L);
	fillMsg(msg_log.conv0, tmp_conversion0);
	fillMsg(msg_log.conv1, tmp_conversion1);
	fillMsg(msg_log.conv2, tmp_conversion2);
	fillMsg(msg_log.s, s);
	fillMsgLink(msg_log.link1, param_REG.segment(0, PARAM));
	fillMsgLink(msg_log.link2, param_REG.segment(10, PARAM));
	fillMsgLink(msg_log.link3, param_REG.segment(20, PARAM));
	fillMsgLink(msg_log.link4, param_REG.segment(30, PARAM));
	fillMsgLink(msg_log.link5, param_REG.segment(40, PARAM));
	fillMsgLink(msg_log.link6, param_REG.segment(50, PARAM));
	fillMsgLink(msg_log.link7, param_REG.segment(60, PARAM));
	fillMsg(msg_log.Yr_param, Yr*param_REG);
	fillMsg(msg_log.tau_cmd, tau_cmd);
	fillMsg(msg_log.tau_tilde, tau_tilde);
	fillMsg(msg_log.ee_tr, ee_tr);
	fillMsg(msg_log.e_kin, e_kin);
	this->pub_log.publish(msg_log);
}

	msg_config.header.stamp  = time_now;
	msg_config.xyz.x = T0EE(0,3);
	msg_config.xyz.y = T0EE(1,3);
	msg_config.xyz.z = T0EE(2,3);

	this->pub_config_.publish(msg_config);
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

	/* if(fabs(tau_d_saturated[i])>tau_limit[i]){
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
	// quaternion to matrix
	// Eigen::Quaterniond quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
	// ee_rot_cmd = rotation_matrix;
	// ee_ang_vel_cmd << msg->ang_vel.x, msg->ang_vel.y, msg->ang_vel.z;
	// ee_ang_acc_cmd << msg->ang_acc.x, msg->ang_acc.y, msg->ang_acc.z;
}

void Backstepping::setFlagUpdate(const panda_controllers::flag::ConstPtr& msg){
	update_kin_flag = msg->flag;
	update_dyn_flag = msg->flag;
}

// void Backstepping::setKinUpdate(const panda_controllers::flag::ConstPtr& msg){
// 	update_dyn_flag = msg->flag;
// }

template <size_t N>
void Backstepping::fillMsg(boost::array<float, N>& msg_, const Eigen::MatrixXd& data_) {
    
	int dim = data_.size();
    for (int i = 0; i < dim; i++) {
        msg_[i] = data_(i);
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
