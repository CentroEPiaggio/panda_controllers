#include <panda_controllers/project_impedance_controller.h>
#include "utils/Jacobians_ee.h"
#include "utils/FrictionTorque.h"
#include <cmath>
#include <math.h>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "utils/pseudo_inversion.h"

#define   	EE_X  		0
#define   	EE_Y  		0
#define   	EE_Z  		0
#define 	K_INIT_POS	300
#define 	K_INIT_OR	1000
#define 	COLL_LIMIT	100
#define 	JOINT_STIFF	{3000, 3000, 3000, 3000, 3000, 2000, 100}
//#define 	COLL_LIMIT	2000

/* 
da fare:
  provare il debug
  provare ad usare una massa desiderata come diagonale della massa vera
  provare tau_fric
  controllare bene il controllo
*/

namespace panda_controllers {

extern std::string name_space;

//------------------------------------------------------------------------------//
//                          		INIT										//
//------------------------------------------------------------------------------//
bool ProjectImpedanceController::init(  hardware_interface::RobotHW* robot_hw, 
                                        ros::NodeHandle& node_handle) {
	// Name space extraction for add a prefix to the topic name
	int n = 0;
	name_space = node_handle.getNamespace();
	n = name_space.find("/", 2);
	name_space = name_space.substr(0,n);


	//--------------- INITIALIZE SUBSCRIBERS AND PUBLISHERS -----------------//

	sub_des_traj_proj_ =  node_handle.subscribe(  "/project_impedance_controller/desired_project_trajectory", 1, 
													&ProjectImpedanceController::desiredProjectTrajectoryCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());

	sub_des_imp_proj_ =   node_handle.subscribe(  "/project_impedance_controller/desired_impedance_project", 1, 
													&ProjectImpedanceController::desiredImpedanceProjectCallback, this,
													ros::TransportHints().reliable().tcpNoDelay());

	sub_ext_forces =      node_handle.subscribe(  "/franka_state_controller/F_ext", 1, 
													&ProjectImpedanceController::f_ext_Callback, this,
													ros::TransportHints().reliable().tcpNoDelay());

	pub_pos_error =         node_handle.advertise<geometry_msgs::TwistStamped>(name_space+"/pos_error", 1);
	pub_cmd_force =         node_handle.advertise<geometry_msgs::WrenchStamped>(name_space+"/cmd_force", 1);
	pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>(name_space+"/franka_ee_pose", 1);
	pub_robot_state_ =      node_handle.advertise<panda_controllers::RobotState>(name_space+"/robot_state", 1);
	pub_impedance_ =        node_handle.advertise<std_msgs::Float64>(name_space+"/current_impedance", 1);
	pub_info_debug =        node_handle.advertise<panda_controllers::InfoDebug>(name_space+"/info_debug", 1);


	//---------------- INITIALIZE SERVICE CLIENTS ------------------//

	collBehaviourClient = node_handle.serviceClient<franka_msgs::SetFullCollisionBehavior>( name_space 
																							+ "/franka_control/set_full_collision_behavior");

	jointImpedanceClient = node_handle.serviceClient<franka_msgs::SetJointImpedance>( name_space 
																						+ "/franka_control/set_joint_impedance");


	//----- INITIALIZE NODE, ROBOT HANDLER AND ROBOT INTERFACE -----//

	std::string arm_id;
	if (!node_handle.getParam("arm_id", arm_id)) {
		ROS_ERROR_STREAM("ProjectImpedanceController: Could not read parameter arm_id");
		return false;
	}
	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		ROS_ERROR(
				"ProjectImpedanceController: Invalid or no joint_names parameters provided, "
				"aborting controller init!");
		return false;
	}
	if (!node_handle.getParam("var_damp", var_damp)) {
		ROS_ERROR_STREAM("ProjectImpedanceController: Could not read parameter var_damp");
		return false;
	}

	franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
		ROS_ERROR_STREAM("ProjectImpedanceController: Error getting model interface from hardware");
		return false;
	}
	try {
		model_handle_.reset(
				new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
				"ProjectImpedanceController: Exception getting model handle from interface: "
				<< ex.what());
		return false;
	}

	franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
		ROS_ERROR_STREAM("ProjectImpedanceController: Error getting state interface from hardware");
		return false;
	}
	try {
		state_handle_.reset(
				new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
				"ProjectImpedanceController: Exception getting state handle from interface: "
				<< ex.what());
		return false;
	}

	hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	if (effort_joint_interface == nullptr) {
		ROS_ERROR_STREAM("ProjectImpedanceController: Error getting effort joint interface from hardware");
		return false;
	}
	for (size_t i = 0; i < 7; ++i) {
		try {
			joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			ROS_ERROR_STREAM(
					"ProjectImpedanceController: Exception getting joint handles: " << ex.what());
		return false;
		}
	}


	//---------------- INITIALIZE VARIABLES ------------------//

	position_d_.setZero();                  // desired position
	or_des.setZero();                       // desired orientation
	dpose_d_.setZero();                     // desired position velocity
	ddpose_d_.setZero();                    // desired acceleration
	F_ext.setZero();                        // measured external force
	cartesian_stiffness_.setIdentity();		// stiffness matrix
	cartesian_damping_.setIdentity();		// damping matrix
	cartesian_mass_.setIdentity();			// virtual cartesian matrix   1 Kg

	cartesian_stiffness_.topLeftCorner(3, 3) 		<< 	K_INIT_POS*Eigen::Matrix3d::Identity();
	cartesian_stiffness_.bottomRightCorner(3, 3) 	<< 	K_INIT_OR*Eigen::Matrix3d::Identity();
	cartesian_damping_.topLeftCorner(3, 3) 			<< 	2.0 * sqrt(K_INIT_POS)*Eigen::Matrix3d::Identity();
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

void ProjectImpedanceController::starting(const ros::Time& /*time*/) {
  
	franka::RobotState initial_state = state_handle_->getRobotState();
	Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
	Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

	// set equilibrium point to current state
	position_d_ = initial_transform.translation();

	// define R matrix for orientation
	Eigen::Matrix<double, 3, 3> R(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()).topLeftCorner(3,3));

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

void ProjectImpedanceController::update(  const ros::Time& /*time*/,
                                          const ros::Duration& /*period*/) {

	//----------- VARIABLES DECLARATIONS and DEFINITIONS -------------//

	// Control
	double tau_fric_array[7];       		// joint friction torque [Cognetti]
	double ja_array[42];                	// Ja array
	double ja_dot_array[42];            	// Ja_dot array
	Eigen::Matrix<double, 6, 1> error;  	// pose error 6x1: position error (3x1) [m] - orientation error XYZ [rad]
	Eigen::Matrix<double, 6, 1> derror; 	// pose vel. error 6x1: vel. error (3x1) [m] - orientation vel. error IN XYZ rate
	Eigen::Matrix<double, 7, 1> tau_fric;    	// joint friction forces vector
	Eigen::Matrix<double, 6, 7> ja;			// analytic Jacobian
	Eigen::Matrix<double, 6, 7> ja_dot;		// analytic Jacobian dot
	Eigen::Matrix<double, 3, 1> or_proj;	// current orientation
	Eigen::Matrix<double, 6, 1> dpose;		// current pose velocity

	// Franka
	franka::RobotState robot_state = state_handle_->getRobotState();        // robot state
	std::array<double, 49> mass_array = model_handle_->getMass();			// mass matrix array
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis();	// coreolis vector
	
	// Eigen conversion
	Eigen::Map<Eigen::Matrix<double, 7, 7> > mass(mass_array.data());                 // mass matrix [kg]
	Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());         // coriolis forces  [Nm]
	Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());                 // joint positions  [rad]
	Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());               // joint velocities [rad/s]
	Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());     // previous cycle commanded torques [Nm]

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

	// GABICCINI (ZYX)
	double phi = atan2(R(1,0),R(0,0));                          
	double theta = atan2(-R(2,0),sqrt(pow(R(2,1),2) + pow(R(2,2),2)));
	double psi = atan2(R(2,1),R(2,2));

	// orientation // GABICCINI (ZYX) but appended as (x=Z, y=Y, z=X)
	or_proj << phi, theta, psi;


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

	
	//===========================| IMPEDANCE CONTROL 6 dof|=============================//

	// SETTING Fext to ZERO!!!   ---------------------------------------------------------------------------   REMOVE THIS!
	F_ext.setZero();


	//----------- VARIABLES -------------//

	//Eigen::VectorXd wrench_task(6), tau_task(7), tau_nullspace(7), tau_d(7); 	// replaced to avoid dynamic allocation

	Eigen::Matrix <double, 6, 1> wrench_task;	// task space wrench
	Eigen::Matrix <double, 7, 1> tau_task;		// tau for primary task
	Eigen::Matrix <double, 7, 1> tau_nullspace;	// tau for nullspace task
	Eigen::Matrix <double, 7, 1> tau_d;			// final desired torque
	Eigen::Matrix <double, 6, 7> ja_t_inv;		// jacobian traspose pseudoiverse (mass weighted)
	Eigen::Matrix <double, 7, 7> N;				// null projector
	Eigen::Matrix <double, 6, 6> task_mass;		// mass in task space


	// define robot mass in task space and NaN substitution
	task_mass << (ja*mass.inverse()*ja.transpose()).inverse();    // 6x6
	for (int i=0; i<6; i++){
		for (int j=0; j<6; j++){
			if (std::isnan(task_mass(i,j))){
				task_mass(i,j) = 1;
			}
		}
	}

	//---------------- CONTROL COMPUTATION -----------------//

	// project impedance controller
	// from matalb: Bx*ddzdes - Bx*inv(Bm)*(Dm*e_dot + Km*e) + (Bx*inv(Bm) - I)*F_ext - Bx*Ja_dot*q_dot;
	wrench_task <<  task_mass*ddpose_d_ 
					- (task_mass*cartesian_mass_.inverse())*(cartesian_damping_*derror + cartesian_stiffness_*error)
					+ (task_mass*cartesian_mass_.inverse() - Eigen::MatrixXd::Identity(6, 6))*F_ext
					- task_mass*ja_dot*dq;

	// from matlab: tau = (Ja')*F_tau + S*q_dot + G + tau_fric;
	// final tau in joint space for primary task
	tau_task << ja.transpose()*wrench_task 
				+ coriolis + tau_fric;

	//null projection
	ja_t_inv << (ja*mass.inverse()*ja.transpose()).inverse()*ja*mass.inverse();
	N << Eigen::MatrixXd::Identity(7, 7) - ja.transpose()*ja_t_inv;

	tau_nullspace <<  N * ( nullspace_stiffness_ * (q_d_nullspace_ - q)       // proportional term
							- (2.0 * sqrt(nullspace_stiffness_)) * dq);       // derivative term

	// Desired torque
	// tau_d << tau_task + tau_nullspace;
	tau_d << tau_task;
	
	//=================================| END CONTROL |==================================//



	//======================| IMPEDANCE POSITION, PD ORIENTATION |======================//
	// not implemented
	//
	//=================================| END CONTROL |==================================//
	//


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

	Eigen::Matrix<double,6,1> temp_err_o;
	temp_err_o << 0, 0, 0, error(3), error(4), error(5);
	Eigen::Matrix<double,6,1> temp_err_p;
	temp_err_p << error(0), error(1), error(2), 0, 0, 0;
	Eigen::Matrix<double,7,1> Mo;
	Mo << ja.transpose()*(task_mass*cartesian_mass_.inverse())*cartesian_stiffness_*temp_err_o;
	Eigen::Matrix<double,7,1> Mp;
	Mp << ja.transpose()*(task_mass*cartesian_mass_.inverse())*cartesian_stiffness_*temp_err_p;


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
	force_cmd_msg.wrench.torque.x = wrench_task(3);
	force_cmd_msg.wrench.torque.y = wrench_task(4);
	force_cmd_msg.wrench.torque.z = wrench_task(5);

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
	}
	for (int i = 0; i <6; i++){
		for (int j = 0; j<6; j++){
			info_debug_msg.cartesian_stiffness[i*6+j] = cartesian_stiffness_(i,j);
		}
	}
	pub_info_debug.publish(info_debug_msg);


	//----------- CURRENT IMPEDANCE -------------//

	std_msgs::Float64 impedance_msg;
	impedance_msg.data = std::pow(cartesian_stiffness_.norm(),2) + std::pow(cartesian_damping_.norm(),2);

	pub_impedance_.publish(impedance_msg);
}


//---------------------------------------------------------------//
//                    	TORQUE SATURATION                    	 //
//---------------------------------------------------------------//
Eigen::Matrix<double, 7, 1> ProjectImpedanceController::saturateTorqueRate(
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
void ProjectImpedanceController::desiredImpedanceProjectCallback(
  		const panda_controllers::DesiredImpedance::ConstPtr& msg){

	for (int i=0; i<6; i++){
		for (int j=0; j<6; j++){
			cartesian_stiffness_(i, j) = msg->stiffness_matrix[i*6 + j];
			cartesian_damping_(i, j) = msg->damping_matrix[i*6 + j];
		}
	}
	// std::cout<< "cartesian_stiffness "<< cartesian_stiffness_ << std::endl;
}


//----------- DESIRED TRAJECTORY -------------//
void ProjectImpedanceController::desiredProjectTrajectoryCallback(
    	const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
  
	position_d_ 	<< msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
	or_des 			<< msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
	dpose_d_ 		<< msg->velocity.position.x, msg->velocity.position.y, msg->velocity.position.z, 
						msg->velocity.orientation.x, msg->velocity.orientation.y, msg->velocity.orientation.z;
	ddpose_d_ 		<< msg->acceleration.position.x, msg->acceleration.position.y, msg->acceleration.position.z, 
						msg->acceleration.orientation.x, msg->acceleration.orientation.y, msg->acceleration.orientation.z;
}

//----------- EXTERNAL FORCES -------------//
void ProjectImpedanceController::f_ext_Callback(const geometry_msgs::WrenchStampedConstPtr& msg){
  	F_ext << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, 0, 0, 0;
}

}  // end namespace franka_softbots


PLUGINLIB_EXPORT_CLASS(panda_controllers::ProjectImpedanceController,
                       controller_interface::ControllerBase)
