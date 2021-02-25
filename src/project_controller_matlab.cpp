#include <panda_controllers/project_controller_matlab.h>
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
#define 	PD_K_OR		1000
#define 	PD_D_OR		2.0*sqrt(PD_K_OR)
#define 	COLL_LIMIT	100
#define 	NULL_STIFF	100
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


//------------------------------------------------------------------------------//
//                          		INIT										//
//------------------------------------------------------------------------------//
bool ProjectImpedanceControllerMatlab::init(ros::NodeHandle& node_handle) {
	// Name space extraction for add a prefix to the topic name


	//--------------- INITIALIZE SUBSCRIBERS AND PUBLISHERS -----------------//
	
	sub_inputs_from_matlab = node_handle.subscribe("/topic_matlab/inputs_from_matlab",1, 
													&ProjectImpedanceControllerMatlab::inputs_from_matlab_Callback, this,
													ros::TransportHints().reliable().tcpNoDelay());


	pub_info_debug =        node_handle.advertise<panda_controllers::InfoDebug>("/topic_matlab/info_debug", 1);
	pub_outputs_to_matlab = node_handle.advertise<panda_controllers::OutputsToMatlab>("/topic_matlab/outputs_to_matlab",1);

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

	return true;
}



//------------------------------------------------------------------------------//
//                          	  STARTING										//
//------------------------------------------------------------------------------//

void ProjectImpedanceControllerMatlab::starting() {


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

}



//------------------------------------------------------------------------------//
//                          	    UPDATE										//
//------------------------------------------------------------------------------//

void ProjectImpedanceControllerMatlab::update() {

	//----------- VARIABLES DECLARATIONS and DEFINITIONS -------------//

	// Control
	double tau_fric_array[7];       		// joint friction torque [Cognetti]
	double ja_array[42];                	// Ja array
	double ja_dot_array[42];            	// Ja_dot array
	Eigen::Matrix<double, 6, 1> error;  	// pose error 6x1: position error (3x1) [m] - orientation error XYZ [rad]
	Eigen::Matrix<double, 6, 1> derror; 	// pose vel. error 6x1: vel. error (3x1) [m] - orientation vel. error IN XYZ rate
	Eigen::Matrix<double, 6, 7> ja;			// analytic Jacobian
	Eigen::Matrix<double, 6, 7> ja_dot;		// analytic Jacobian dot
	Eigen::Matrix<double, 3, 1> or_proj;	// current orientation
	Eigen::Matrix<double, 6, 1> dpose;		// current pose velocity


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


	// GABICCINI (ZYX)
	double phi = atan2(R(1,0),R(0,0));                          
	double theta = atan2(-R(2,0),sqrt(pow(R(2,1),2) + pow(R(2,2),2)));
	double psi = atan2(R(2,1),R(2,2));

	// orientation // GABICCINI (ZYX) but appended as (x=Z, y=Y, z=X)
	or_proj << phi, theta, psi;


	//----------- COMPUTE ERRORS -------------//

	// position error expressed in the base frame
	error.head(3) << pos - position_d_;
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
	// tau_task << ja.transpose()*wrench_task 
	// 			+ coriolis + tau_fric;

	// control for matlab
	tau_task << ja.transpose()*wrench_task 
				+ coriolis + tau_fric + gravity;

	//null projection
	ja_t_inv << (ja*mass.inverse()*ja.transpose()).inverse()*ja*mass.inverse();
	N << Eigen::MatrixXd::Identity(7, 7) - ja.transpose()*ja_t_inv;

	tau_nullspace <<  N * ( nullspace_stiffness_ * (q_d_nullspace_ - q)       // proportional term
							- (2.0 * sqrt(nullspace_stiffness_)) * dq);       // derivative term

	// Desired torque
	// tau_d << tau_task + tau_nullspace;
	tau_d << tau_task;
	
	//=================================| END CONTROL |==================================//



	//----------- TORQUE SATURATION and COMMAND-------------//
	
	// Saturate torque to avoid torque limit
	double ith_torque_rate;
	for (int i = 0; i < 7; ++i){
		ith_torque_rate = std::abs(tau_d(i)/tau_limit(i));
		if( ith_torque_rate > 1)
			tau_d = tau_d / ith_torque_rate;
	}



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


	//-------------Matlab_Publisher-----------//
	for(int i=0; i<7; i++){
		outputs_to_matlab_msg.tau[i] = tau_d(i);
	}
	pub_outputs_to_matlab.publish(outputs_to_matlab_msg);

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
		info_debug_msg.tau_fric[i] = tau_fric(i);
	}
	for (int i = 0; i <6; i++){
		for (int j = 0; j<6; j++){
			info_debug_msg.cartesian_stiffness[i*6+j] = cartesian_stiffness_(i,j);
		}
	}
	pub_info_debug.publish(info_debug_msg);

}



//---------------------------------------------------------------//
//                          CALLBACKS		                     //
//---------------------------------------------------------------//

//----------- For Matlab -------------//
void ProjectImpedanceControllerMatlab::inputs_from_matlab_Callback(const panda_controllers::InputsFromMatlabConstPtr& msg){
	position_d_ << msg->pose_des[0], msg->pose_des[1], msg->pose_des[2];
	or_des << msg->pose_des[3], msg->pose_des[4], msg->pose_des[5];
	dpose_d_ << msg->dpose_des[0], msg->dpose_des[1], msg->dpose_des[2], msg->dpose_des[3], msg->dpose_des[4], msg->dpose_des[5];
	ddpose_d_ << msg->ddpose_des[0], msg->ddpose_des[1], msg->ddpose_des[2], msg->ddpose_des[3], msg->ddpose_des[4], msg->ddpose_des[5];
	F_ext << msg->F_ext[0], msg->F_ext[1], msg->F_ext[2], msg->F_ext[3], msg->F_ext[4], msg->F_ext[5];
	q << msg->q[0], msg->q[1], msg->q[2], msg->q[3], msg->q[4], msg->q[5], msg->q[6];
	dq << msg->dq[0], msg->dq[1], msg->dq[2], msg->dq[3], msg->dq[4], msg->dq[5], msg->dq[6]; 
	pos << msg->pos[0], msg->pos[1], msg->pos[2];


	for(int i=0; i<6; i++){
		for(int j=0; j<6; j++){
			cartesian_stiffness_(i,j) = msg->K[i*6+j];
			cartesian_damping_(i,j) = msg->D[i*6+j];  
		}
	}

	Eigen::Matrix <double, 7, 7> Coriolis_temp;

	for(int i=0; i<7; i++){
		tau_fric(i) = msg->tau_fric_mat[i];
		gravity(i) = msg->gravity_mat[i];
		for(int j=0; j<7; j++){
			mass(i,j) = msg->mass_mat[i*7+j];
			Coriolis_temp(i,j) = msg->coriolis_mat[i*7+j];
		}
	}
	coriolis = Coriolis_temp*dq;

	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			R(i,j) = msg->R[i*3+j];
		}
	}

}

}  // end namespace franka_softbots

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  ros::NodeHandle node_handle;

  ros::Rate loop_rate(100);

  panda_controllers::ProjectImpedanceControllerMatlab controller;

  controller.init(node_handle);
  ros::spinOnce();
  controller.starting();

  while (ros::ok()){

    ros::spinOnce();
	controller.update();

    loop_rate.sleep();
  }

  return 0;
}