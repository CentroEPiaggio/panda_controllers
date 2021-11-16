#include <panda_controllers/impedance_controller.h>
#include "utils/Jacobians_ee.h"
#include "utils/FrictionTorque.h"
#include <cmath>
#include <math.h>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "utils/pseudo_inversion.h"

#define   	EE_X  			0
#define   	EE_Y  			0
#define   	EE_Z  			0.18		// m  0.18
#define 	K_INIT_POS		200
#define 	K_INIT_OR		500
#define 	PD_K_OR			50
#define 	PD_D_OR			2.0*sqrt(PD_K_OR)
#define 	PD_K_OR_QUAT_X	30
#define 	PD_K_OR_QUAT_Y	30
#define 	PD_K_OR_QUAT_Z	30
#define 	PD_D_OR_QUAT_X	2.0*sqrt(PD_K_OR_QUAT_X)
#define 	PD_D_OR_QUAT_Y	2.0*sqrt(PD_K_OR_QUAT_Y)
#define 	PD_D_OR_QUAT_Z	2.0*sqrt(PD_K_OR_QUAT_Z)
#define 	COLL_LIMIT		25
#define 	NULL_STIFF		10
#define 	JOINT_STIFF		{3000, 3000, 3000, 3000, 3000, 2000, 100}
//#define 	COLL_LIMIT		2000

#define 	USE_FILTER		0
#define 	BETA_FILTER		0.001*0.1
#define 	ALPHA_FILTER	0.1*0.1

/*
da fare:
  //aggiungere matrice di Rotazione
*/

namespace panda_controllers {

extern std::string name_space;

//------------------------------------------------------------------------------//
//                          		INIT										//
//------------------------------------------------------------------------------//
bool ImpedanceController::init(  hardware_interface::RobotHW* robot_hw,
                                        ros::NodeHandle& node_handle) {
	// Name space extraction for add a prefix to the topic name
	int n = 0;
	name_space = node_handle.getNamespace();
	n = name_space.find("/", 2);
	name_space = name_space.substr(0,n);


	//--------------- INITIALIZE SUBSCRIBERS AND PUBLISHERS -----------------//

	sub_des_traj_proj_ =  node_handle.subscribe(  "/project_impedance_controller/desired_project_trajectory", 1,
													&ImpedanceController::desiredTrajectoryCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());

	sub_des_imp_proj_ =   node_handle.subscribe(  "/project_impedance_controller/desired_impedance_project", 1,
													&ImpedanceController::desiredImpedanceCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());

	// sub_ext_forces =      node_handle.subscribe(  "/franka_state_controller/F_ext", 1,
	// 												&ImpedanceController::f_ext_Callback, this,
	// 												ros::TransportHints().reliable().tcpNoDelay());


	sub_ext_forces =      node_handle.subscribe(  "/my_sensor_right/ft_sensor_hw/my_sensor_right", 1,
	 												&ImpedanceController::f_ext_Callback, this,
	 												ros::TransportHints().reliable().tcpNoDelay());

	pub_pos_error =         node_handle.advertise<geometry_msgs::TwistStamped>("/project_impedance_controller/pos_error", 1);
	pub_cmd_force =         node_handle.advertise<geometry_msgs::WrenchStamped>("/project_impedance_controller/cmd_force", 1);
	pub_ext_forces =        node_handle.advertise<geometry_msgs::WrenchStamped>("/project_impedance_controller/ext_forces", 1);
	pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/project_impedance_controller/franka_ee_pose", 1);
	pub_robot_state_ =      node_handle.advertise<panda_controllers::RobotState>("/project_impedance_controller/robot_state", 1);
	pub_impedance_ =        node_handle.advertise<std_msgs::Float64>("/project_impedance_controller/current_impedance", 1);
	pub_info_debug =        node_handle.advertise<panda_controllers::InfoDebug>("/project_impedance_controller/info_debug", 1);


	//---------------- INITIALIZE SERVICE CLIENTS ------------------//

	collBehaviourClient = node_handle.serviceClient<franka_msgs::SetFullCollisionBehavior>( name_space
																							+ "/franka_control/set_full_collision_behavior");

	jointImpedanceClient = node_handle.serviceClient<franka_msgs::SetJointImpedance>( name_space
																						+ "/franka_control/set_joint_impedance");


	//----- INITIALIZE NODE, ROBOT HANDLER AND ROBOT INTERFACE -----//

	std::string arm_id;
	if (!node_handle.getParam("arm_id", arm_id)) {
		ROS_ERROR_STREAM("ImpedanceController: Could not read parameter arm_id");
		return false;
	}
	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		ROS_ERROR(
				"ImpedanceController: Invalid or no joint_names parameters provided, "
				"aborting controller init!");
		return false;
	}
	if (!node_handle.getParam("var_damp", var_damp)) {
		ROS_ERROR_STREAM("ImpedanceController: Could not read parameter var_damp");
		return false;
	}

	franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
		ROS_ERROR_STREAM("ImpedanceController: Error getting model interface from hardware");
		return false;
	}
	try {
		model_handle_.reset(
				new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
				"ImpedanceController: Exception getting model handle from interface: "
				<< ex.what());
		return false;
	}

	franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
		ROS_ERROR_STREAM("ImpedanceController: Error getting state interface from hardware");
		return false;
	}
	try {
		state_handle_.reset(
				new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
				"ImpedanceController: Exception getting state handle from interface: "
				<< ex.what());
		return false;
	}

	hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	if (effort_joint_interface == nullptr) {
		ROS_ERROR_STREAM("ImpedanceController: Error getting effort joint interface from hardware");
		return false;
	}
	for (size_t i = 0; i < 7; ++i) {
		try {
			joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			ROS_ERROR_STREAM(
					"ImpedanceController: Exception getting joint handles: " << ex.what());
		return false;
		}
	}


	//---------------- INITIALIZE VARIABLES ------------------//

	position_d_.setZero();                  // desired position
	or_des.setZero();                       // desired orientation
	dpose_d_.setZero();                     // desired position velocity
	ddpose_d_.setZero();                    // desired acceleration
	F_ext.setZero();                        // measured external force
	// F_bias.setZero();						// force by low-pass filter
	F_global.setZero();						// force by force sensor
	cartesian_stiffness_.setIdentity();		// stiffness matrix
	cartesian_damping_.setIdentity();		// damping matrix
	cartesian_mass_.setIdentity();			// virtual cartesian matrix   1 Kg
	orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;           // desired orientation

	cartesian_stiffness_.topLeftCorner(3, 3) 		<< 	K_INIT_POS*Eigen::Matrix3d::Identity();
	// cartesian_stiffness_(2,2) 						=   200;
	cartesian_stiffness_.bottomRightCorner(3, 3) 	<< 	K_INIT_OR*Eigen::Matrix3d::Identity();
	cartesian_damping_.topLeftCorner(3, 3) 			<< 	2.0 * sqrt(K_INIT_POS)*Eigen::Matrix3d::Identity();	// Eigen::Matrix3d::Identity(3,3)*200;
	cartesian_damping_.bottomRightCorner(3, 3) 		<< 	2.0 * sqrt(K_INIT_OR)*Eigen::Matrix3d::Identity();

	tau_limit << 87, 87, 87, 87, 12, 12, 12;  //joint torques limit vector

	// Collision behaviours limits
	collBehaviourSrvMsg.request.lower_torque_thresholds_acceleration 	= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.upper_torque_thresholds_acceleration 	= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.lower_torque_thresholds_nominal 		= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.upper_torque_thresholds_nominal 		= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.lower_force_thresholds_acceleration 	= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.upper_force_thresholds_acceleration 	= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.lower_force_thresholds_nominal 			= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};
	collBehaviourSrvMsg.request.upper_force_thresholds_nominal 			= {COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT, COLL_LIMIT};

	// Joint impedance
	jointImpedanceSrvMsg.request.joint_stiffness = JOINT_STIFF;

	return true;
}



//------------------------------------------------------------------------------//
//                          	  STARTING										//
//------------------------------------------------------------------------------//

void ImpedanceController::starting(const ros::Time& /*time*/) {

	franka::RobotState initial_state = state_handle_->getRobotState();
	Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
	Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

	// set equilibrium point to current state
	position_d_ = initial_transform.translation();

	// define R matrix for orientation
	Eigen::Matrix<double, 3, 3> R(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()).topLeftCorner(3,3));

	orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

	double phi 		= atan2(R(1,0),R(0,0));
	double theta 	= atan2(-R(2,0),sqrt(pow(R(2,1),2) + pow(R(2,2),2)));
	double psi 		= atan2(R(2,1),R(2,2));

	// orientation // GABICCINI (ZYX) but appended as (x=Z, y=Y, z=X)
	or_des << phi, theta, psi;

	// set nullspace equilibrium configuration to central angles of joints
	Eigen::Matrix <double, 7, 1> q_min;
	Eigen::Matrix <double, 7, 1> q_max;
	q_min << -2.8973, -1.7628, -2.8973, -3.0718+0.1745, -2.8973, -0.0175+0.0873, -2.8973;
	q_max <<  2.8973,  1.7628,  2.8973, -0.0698-0.1745,  2.8973,  3.7525-0.0873,  2.8973;
	q_d_nullspace_ << (q_max + q_min)/2;

	// set collision behaviour
	collBehaviourClient.call(collBehaviourSrvMsg);

	// set joint impedance
	jointImpedanceClient.call(jointImpedanceSrvMsg);
}



//------------------------------------------------------------------------------//
//                          	    UPDATE										//
//------------------------------------------------------------------------------//

void ImpedanceController::update(  const ros::Time& /*time*/,
                                          const ros::Duration& /*period*/) {

	//----------- VARIABLES DECLARATIONS and DEFINITIONS -------------//

	// Control
	double tau_fric_array[7];       		// joint friction torque [Cognetti]
	double ja_array[42];                	// Ja array
	double ja_dot_array[42];            	// Ja_dot array
	Eigen::Matrix<double, 6, 1> error;  	// pose error 6x1: position error (3x1) [m] - orientation error XYZ [rad]
	Eigen::Matrix<double, 6, 1> derror; 	// pose vel. error 6x1: vel. error (3x1) [m] - orientation vel. error IN XYZ rate
	Eigen::Matrix<double, 7, 1> tau_fric;   // joint friction forces vector
	Eigen::Matrix<double, 6, 7> ja;			// analytic Jacobian
	Eigen::Matrix<double, 6, 7> ja_dot;		// analytic Jacobian dot
	Eigen::Matrix<double, 3, 1> or_proj;	// current orientation
	Eigen::Matrix<double, 6, 1> dpose;		// current pose velocity

	// Franka
	franka::RobotState robot_state = state_handle_->getRobotState();        // robot state
	std::array<double, 49> mass_array = model_handle_->getMass();			// mass matrix array
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis();	// coreolis vector
	std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

	// Eigen conversion
	Eigen::Map<Eigen::Matrix<double, 7, 7> > mass(mass_array.data());                 // mass matrix [kg]
	Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());         // coriolis forces  [Nm]
	Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());                 // joint positions  [rad]
	Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());               // joint velocities [rad/s]
	Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());     // previous cycle commanded torques [Nm]
	Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());         // jacobian

	Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));       // ee-base homog. transf. matrix
	Eigen::Vector3d position(transform.translation());                                // ee-base position [m]
	Eigen::Quaterniond orientation(transform.linear());                               // ee-base orientation


	//----------- PUBLISH MATRICES FOR PLANNING -------------//

	// publish mass and jacobian
	robot_state_msg.header.stamp = ros::Time::now();
	std::copy(mass_array.begin(), mass_array.end(), robot_state_msg.mass_matrix.begin());

	pub_robot_state_.publish(robot_state_msg);


	//----------- COMPUTE JACOBIANS and FRICTION -------------//

	// Filling arrays
	double q_array[7], dq_array[7];
	for (int i=0; i<7; i++){
		q_array[i] = q(i);
		dq_array[i] = dq(i);
	}

	// Compute Ja and NaN removal
	get_Ja_proj(q_array, EE_X, EE_Y, EE_Z, ja_array);
	for (int i=0; i<6; i++){
		for (int j=0; j<7; j++){
			if (std::isnan(ja_array[i*7+j])){
				ja(i,j) = 1;
			}else {
				ja(i,j) = ja_array[i*7+j];
			}
		}
	}

	// Compute Ja_dot and NaN removal
	get_Ja_dot_proj(q_array, dq_array, EE_X, EE_Y, EE_Z, ja_dot_array);
	for (int i=0; i<6; i++){
		for (int j=0; j<7; j++){
			if (std::isnan(ja_dot_array[i*7+j])){
				ja_dot(i,j) = 1;
			}else {
				ja_dot(i,j) = ja_dot_array[i*7+j];
			}
		}
	}

	// Friction torque
	get_friction_torque(dq_array, tau_fric_array);
	for(int i=0; i<7; i++){
		tau_fric(i) = tau_fric_array[i];
	}


	//----------- COMPUTE ORIENTATION -------------//

	Eigen::Matrix<double, 3, 3> R(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()).topLeftCorner(3,3));
	// Eigen::Matrix<double, 3, 1> F_new;
	// std::cout << "F_local " << F_ext.head(3) << std::endl;
	F_global << R*F_ext.head(3); 				// F_ext in cartesian space
	// std::cout << "F_global " << F_global<< std::endl;
	// GABICCINI (ZYX)
	double phi = atan2(R(1,0),R(0,0));
	double theta = atan2(-R(2,0),sqrt(pow(R(2,1),2) + pow(R(2,2),2)));
	double psi = atan2(R(2,1),R(2,2));

	// orientation // GABICCINI (ZYX) but appended as (x=Z, y=Y, z=X)
	or_proj << phi, theta, psi;


	// //----------- DISSIPATIVE FILTER -------------//

	// Eigen::Matrix<double, 3, 1> F_new_;
	// Eigen::Matrix<double, 3, 1> F_global_;
	// if (USE_FILTER){
	// 	// LOW PASS FILTER
	// 	F_bias << F_bias + BETA_FILTER*(F_new - F_bias);
	// 	// F_bias.setZero();

	// 	// DISSIPATIVE FILTER
	// 	F_new_ << F_new - F_bias;
	// 	F_global_ << F_global - F_bias;
	// 	for (int i=0; i<3; i++){
	// 		if (F_new_[i]*F_global_[i] > 0){
	// 			if (std::abs(F_new_[i]) > std::abs(F_global_[i])){
	// 				F_global[i] = F_global[i] + ALPHA_FILTER * (F_new[i] - F_global[i]);
	// 			}else{
	// 				F_global[i] = F_new[i];
	// 			}
	// 		}else{
	// 			F_global[i] = F_bias[i] + ALPHA_FILTER * (F_new_[i]);
	// 		}
	// 	}
	// }else{
	// 	F_global << F_new;
	// }


	//----------- COMPUTE ERRORS -------------//

	// position error expressed in the base frame
	error.head(3) << position - position_d_;
	error.tail(3) << or_proj - or_des;

	// wrap to PI
	error(3) = atan2(sin(error(3)),cos(error(3)));
	error(4) = atan2(sin(error(4)),cos(error(4)));
	error(5) = atan2(sin(error(5)),cos(error(5)));

	// velocity error
	dpose << ja * dq;
	derror.head(3) << dpose.head(3) - dpose_d_.head(3);
	derror.tail(3) << ja.bottomLeftCorner(3,7)*dq - dpose_d_.tail(3);



	//======================| IMPEDANCE POSITION, PD ORIENTATION |======================//

	// SETTING Fext to ZERO!!!   ---------------------------------------------------------------------------   REMOVE THIS!
	// F_ext.setZero();

	//----------- VARIABLES -------------//

	//Eigen::VectorXd wrench_task(6), tau_task(7), tau_nullspace(7), tau_d(7); 	// replaced to avoid dynamic allocation

	Eigen::Matrix <double, 3, 1> wrench_task;	// task space wrench
	Eigen::Matrix <double, 7, 1> tau_task;		// tau for primary task
	Eigen::Matrix <double, 7, 1> tau_nullspace;	// tau for nullspace task
	Eigen::Matrix <double, 7, 1> tau_d;			// final desired torque
	Eigen::Matrix <double, 3, 7> ja_t_inv;		// jacobian traspose pseudoiverse (mass weighted)
	Eigen::Matrix <double, 7, 7> N;
    Eigen::Matrix <double, 7, 7> N_tot;				// null projector
	Eigen::Matrix <double, 3, 3> task_mass;		// mass in task space

	Eigen::Matrix <double, 3, 7> ja_pos;
	Eigen::Matrix <double, 3, 7> ja_or;
	Eigen::Matrix <double, 3, 7> ja_dot_pos;
	Eigen::Matrix <double, 3, 3> cartesian_mass_pos;
	Eigen::Matrix <double, 3, 3> cartesian_damping_pos;
	Eigen::Matrix <double, 3, 3> cartesian_stiffness_pos;
	Eigen::Matrix <double, 3, 1> derror_pos;
	Eigen::Matrix <double, 3, 1> derror_or;
	Eigen::Matrix <double, 3, 1> error_pos;
	Eigen::Matrix <double, 3, 1> error_or;
	Eigen::Matrix <double, 3, 7> ja_pos_t_inv;
	Eigen::Matrix <double, 6, 7> jacobian_t_inv;
	Eigen::Matrix <double, 7, 1> tau_or;
	Eigen::Matrix <double, 3, 3> I3;
	Eigen::Matrix <double, 7, 3> ja_or_pinv;


	// ja_pos << ja.topLeftCorner(3,7);
	ja_pos << jacobian.topLeftCorner(3,7);
	ja_dot_pos << ja_dot.topLeftCorner(3,7);
	cartesian_damping_pos << cartesian_damping_.topLeftCorner(3,3);
	cartesian_stiffness_pos << cartesian_stiffness_.topLeftCorner(3,3);
	cartesian_mass_pos << 1.0 * Eigen::MatrixXd::Identity(3, 3);
	error_pos << error.head(3);
	derror_pos << derror.head(3);
	error_or << error.tail(3);
	derror_or << derror.tail(3);
	I3 = Eigen::MatrixXd::Identity(3, 3);

	// define robot mass in task space and NaN substitution
	task_mass << (ja_pos*mass.inverse()*ja_pos.transpose()).inverse();    // 3x3
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){
			if (std::isnan(task_mass(i,j))){
				task_mass(i,j) = 1;
			}
		}
	}

	// // To remove
	// cartesian_stiffness_pos(0,0) = 0;
	// cartesian_stiffness_pos(1,1) = 0;
	// cartesian_stiffness_pos(2,2) = 0;
	// cartesian_damping_pos(0,0) = 2*sqrt(4);
	// cartesian_damping_pos(1,1) = 2*sqrt(4);
	// cartesian_damping_pos(2,2) = 2*sqrt(4);

	//---------------- POSITION CONTROL COMPUTATION -----------------//

	// project impedance controller
	// from matalb: Bx*ddzdes - Bx*inv(Bm)*(Dm*e_dot + Km*e) + (Bx*inv(Bm) - I)*F_ext - Bx*Ja_dot*q_dot;
	wrench_task <<  task_mass*ddpose_d_.head(3)
					- (task_mass*cartesian_mass_pos.inverse())*(cartesian_damping_pos*derror_pos + cartesian_stiffness_pos*error_pos)
					+ (task_mass*cartesian_mass_pos.inverse() - I3) * F_global;
					- task_mass*ja_dot_pos*dq;


	// from matlab: tau = (Ja')*F_tau + S*q_dot + G + tau_fric;
	// final tau in joint space for primary task
	tau_task << ja_pos.transpose()*wrench_task
				+ coriolis;  // + tau_fric;


	//---------------- ORIENTATION CONTROL COMPUTATION -----------------//

	//null projection
	ja_pos_t_inv << (ja_pos*ja_pos.transpose()).inverse()*ja_pos;
	// ja_pos_t_inv << (ja_pos*mass.inverse()*ja_pos.transpose()).inverse()*ja_pos*mass.inverse();
	jacobian_t_inv << (jacobian*mass.inverse()*jacobian.transpose()).inverse()*jacobian*mass.inverse();
	ja_or_pinv << (ja_or.transpose()*ja_or).inverse()*ja_or.transpose();
	N << Eigen::MatrixXd::Identity(7, 7) - ja_pos.transpose()*ja_pos_t_inv;
	N_tot << Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose()*jacobian_t_inv;

	//-----Quaternion orientation-----//
	Eigen::Matrix<double, 6, 1> dposition;
	Eigen::Matrix<double,3,1> error_quat;
	Eigen::Matrix<double,3,1> derror_quat;
	Eigen::Matrix<double, 3, 1> wrench_task_or;

	// orientation error expressed in the base frame
	if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
		orientation.coeffs() << -orientation.coeffs();

	// "difference" quaternion
	Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
	// convert to axis angle
	Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
	// limit orientation error amplitude to avoid bad robot behaviour
	while( abs(error_quaternion_angle_axis.angle()) > M_PI/2){
		if ( error_quaternion_angle_axis.angle() > M_PI/2 )
			error_quaternion_angle_axis.angle() = error_quaternion_angle_axis.angle() - M_PI/2;
		else if ( error_quaternion_angle_axis.angle() < -M_PI/2)
			error_quaternion_angle_axis.angle() = error_quaternion_angle_axis.angle() + M_PI/2;
	}
	error_quat << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

	dposition << jacobian * dq;

	derror_quat << dposition.tail(3);

	Eigen::Matrix<double, 3, 3> K_orient;
	K_orient.setZero();
	K_orient(0,0) = PD_K_OR_QUAT_X;
	K_orient(1,1) = PD_K_OR_QUAT_Y;
	K_orient(2,2) = PD_K_OR_QUAT_Z;

	Eigen::Matrix<double, 3, 3> D_orient;
	D_orient.setZero();
	D_orient(0,0) = PD_D_OR_QUAT_X;
	D_orient(1,1) = PD_D_OR_QUAT_Y;
	D_orient(2,2) = PD_D_OR_QUAT_Z;

	wrench_task_or << ( - K_orient * error_quat               	  // proportional term tested
                 		- D_orient * derror_quat );               // derivative term

	// PROVE
	tau_or << N * ( jacobian.bottomLeftCorner(3,7).transpose() * wrench_task_or );
	// tau_or = N * ja_or.transpose() * ( -I3*error_or*PD_K_OR - I3*derror_or*PD_D_OR );
	// tau_or = N * ja_or_pinv * ( -I3*error_or*PD_K_OR - I3*derror_or*PD_D_OR );
	// tau_or = ja_or.transpose() * ( -I3*error_or*PD_K_OR - I3*derror_or*PD_D_OR );	// ? NaN coommand action ?

	// FUNZIONANTE
	// tau_or << jacobian.bottomLeftCorner(3,7).transpose() * wrench_task_or; 	// PD_K_OR_QUAT 30

	//---------------- NULLSPACE CONTROL COMPUTATION -----------------//

	 tau_nullspace <<  N_tot * ( NULL_STIFF * (q_d_nullspace_ - q)       // proportional term
						- (2.0 * sqrt(NULL_STIFF)) * dq);       // derivative term

	// std::cout<< "N: " << N_tot << std::endl;
	// std::cout << "dq: " << dq << std::endl;
	// std::cout << "q: " << q << std::endl;

	// Desired torque
	tau_d << tau_task + tau_or; // + tau_nullspace;
	// tau_d << tau_or;

	//=================================| END CONTROL |==================================//



	//----------- TORQUE SATURATION and COMMAND-------------//

	// Saturate torque rate to avoid discontinuities
	tau_d << saturateTorqueRate(tau_d, tau_J_d);

	// Saturate torque to avoid torque limit
	double ith_torque_rate;
	for (int i = 0; i < 7; ++i){
		ith_torque_rate = std::abs(tau_d(i)/tau_limit(i));
		if( ith_torque_rate > 1)
			tau_d = tau_d / ith_torque_rate;
	}

	//set arm command torques
	for (size_t i = 0; i < 7; ++i)
		joint_handles_[i].setCommand(tau_d(i));



	//======================| PUBISH & SUBSCRIBE |======================//

	//----------- DEBUG STUFF -------------//

	Eigen::Matrix<double,7,1> Mo;
	Mo << tau_or;
	Eigen::Matrix<double,7,1> Mp;
	Mp << tau_task;


	//----------- POSE ERROR -------------//

	pos_error_msg.header.stamp = ros::Time::now();
	pos_error_msg.twist.linear.x = -error(0);
	pos_error_msg.twist.linear.y = -error(1);
	pos_error_msg.twist.linear.z = -error(2);
	pos_error_msg.twist.angular.x = -error(3);
	pos_error_msg.twist.angular.y = -error(4);
	pos_error_msg.twist.angular.z = -error(5);

	pub_pos_error.publish(pos_error_msg);


	//----------- COMMANDED WRENCH -------------//

	force_cmd_msg.wrench.force.x = wrench_task(0);
	force_cmd_msg.wrench.force.y = wrench_task(1);
	force_cmd_msg.wrench.force.z = wrench_task(2);
	// force_cmd_msg.wrench.torque.x = wrench_task(3);
	// force_cmd_msg.wrench.torque.y = wrench_task(4);
	// force_cmd_msg.wrench.torque.z = wrench_task(5);

	pub_cmd_force.publish(force_cmd_msg);


	//----------- EE POSE -------------//

	geometry_msgs::PoseStamped postion_endeff;
	postion_endeff.pose.position.x = position.x();
	postion_endeff.pose.position.y = position.y();
	postion_endeff.pose.position.z = position.z();
	postion_endeff.pose.orientation.w = 0.0;
	// GABICCINI (ZYX) but appended as (x=Z, y=Y, z=X)
	postion_endeff.pose.orientation.x = or_proj(0);
	postion_endeff.pose.orientation.y = or_proj(1);
	postion_endeff.pose.orientation.z = or_proj(2);

	pub_endeffector_pose_.publish(postion_endeff);


	//----------- DEBUG INFO -------------//
	info_debug_msg.header.stamp = ros::Time::now();
	info_debug_msg.pose_error.position.y = error[1];
	info_debug_msg.pose_error.position.x = error[0];
	info_debug_msg.pose_error.position.z = error[2];
	info_debug_msg.pose_error.orientation.x = error[5];
	info_debug_msg.pose_error.orientation.y = error[4];
	info_debug_msg.pose_error.orientation.z = error[3];
	for(int i=0; i<7;i++){
		info_debug_msg.tau_pos[i] = Mp(i);
		info_debug_msg.tau_or[i] = Mo(i);
		info_debug_msg.tau_internal[i] = tau_J_d(i);
		info_debug_msg.tau_fric[i] = tau_fric(i);
	    info_debug_msg.tau_null[i] = tau_nullspace(i);

	}
	for (int i = 0; i <3; i++){
		for (int j = 0; j<3; j++){
			info_debug_msg.cartesian_stiffness[i*3+j] = cartesian_stiffness_pos(i,j);
		}
	}
	pub_info_debug.publish(info_debug_msg);

	//----------- F_ext -------------//

	geometry_msgs::WrenchStamped wrench_msg;
	wrench_msg.header.stamp = ros::Time::now();
	wrench_msg.wrench.force.x = F_global(0);
	wrench_msg.wrench.force.y = F_global(1);
	wrench_msg.wrench.force.z = F_global(2);
	pub_ext_forces.publish(wrench_msg);

	//----------- CURRENT IMPEDANCE -------------//

	std_msgs::Float64 impedance_msg;
	impedance_msg.data = std::pow(cartesian_stiffness_pos.norm(),2) + std::pow(cartesian_damping_pos.norm(),2);

	pub_impedance_.publish(impedance_msg);
}


//---------------------------------------------------------------//
//                    	TORQUE SATURATION                    	 //
//---------------------------------------------------------------//
Eigen::Matrix<double, 7, 1> ImpedanceController::saturateTorqueRate(
    	const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    	const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
	Eigen::Matrix<double, 7, 1> tau_d_saturated;
	for (size_t i = 0; i < 7; i++) {
		double difference = tau_d_calculated[i] - tau_J_d[i];
		tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
	}
	return tau_d_saturated;
}


//---------------------------------------------------------------//
//                          CALLBACKS		                     //
//---------------------------------------------------------------//

//----------- DESIRED IMPEDANCE -------------//
void ImpedanceController::desiredImpedanceCallback(
  		const panda_controllers::DesiredImpedance::ConstPtr& msg){

	for (int i=0; i<6; i++){
		for (int j=0; j<6; j++){
			cartesian_stiffness_(i, j) = msg->stiffness_matrix[i*6 + j];
			cartesian_damping_(i, j) = msg->damping_matrix[i*6 + j];
		}
	}
}


//----------- DESIRED TRAJECTORY -------------//
void ImpedanceController::desiredTrajectoryCallback(
    	const panda_controllers::DesiredTrajectoryConstPtr& msg) {

	position_d_ 	<< msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
	or_des 			<< msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
	dpose_d_ 		<< msg->velocity.position.x, msg->velocity.position.y, msg->velocity.position.z,
						msg->velocity.orientation.x, msg->velocity.orientation.y, msg->velocity.orientation.z;
	ddpose_d_ 		<< msg->acceleration.position.x, msg->acceleration.position.y, msg->acceleration.position.z,
						msg->acceleration.orientation.x, msg->acceleration.orientation.y, msg->acceleration.orientation.z;
}

//----------- EXTERNAL FORCES -------------//
void ImpedanceController::f_ext_Callback(const geometry_msgs::WrenchStampedConstPtr& msg){
  	F_ext << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, 0, 0, 0;
}

}  // end namespace franka_softbots


PLUGINLIB_EXPORT_CLASS(panda_controllers::ImpedanceController,
                       controller_interface::ControllerBase)
