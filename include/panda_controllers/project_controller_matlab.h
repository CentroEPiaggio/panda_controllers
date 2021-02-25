// riga 80
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>



#include <ros/node_handle.h>
#include <ros/time.h>
#include <eigen3/Eigen/Dense>



#include <panda_controllers/InfoDebug.h>
#include <panda_controllers/InputsFromMatlab.h>
#include <panda_controllers/OutputsToMatlab.h>
// #include <panda_controllers/DesiredTrajectory.h>


#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

namespace panda_controllers {

//==========================================================================================//
//                                    CLASS CONTROLLER                                      //
//==========================================================================================//
class ProjectImpedanceControllerMatlab  {
 	public:
		bool init(ros::NodeHandle& node_handle);
		void starting();
		void update();

 	private:
		//----------FRANKA STUFF----------//
		Eigen::Matrix<double, 7, 1> saturateTorqueRate(
			const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
			const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

		


		//----------VARIABLES----------//
		bool var_damp;                                            // freely variable damping or critically damped
		double filter_params_{0.1};                               // exponential filter parameter
		double nullspace_stiffness_{50.0};                        // nullspace stiffness [Nm/rad]
		const double delta_tau_max_{1.0};                         // torque rate limit [Nm/ms], from datasheet https://frankaemika.github.io/docs/control_parameters.html
		Eigen::Matrix<double, 7, 1> tau_limit;                    // joint torque limits vector [Nm], from datasheet https://frankaemika.github.io/docs/control_parameters.html
		Eigen::Matrix<double, 6, 6> cartesian_stiffness_;         // actual stiffness matrix
		Eigen::Matrix<double, 6, 6> cartesian_damping_;           // actual damping matrix
		Eigen::Matrix<double, 6, 6> cartesian_mass_;              // desired mass matrix 
		Eigen::Matrix<double, 7, 1> q_d_nullspace_;               // desired joint position (controlled in the nullspace)
		Eigen::Vector3d position_d_;                              // desired position
		Eigen::Vector3d or_des;                                   // orientation for project impedance
		Eigen::Matrix<double, 6, 1> dpose_d_;                     // desired velocity
		Eigen::Matrix<double, 6, 1> ddpose_d_;                    // desired acceleration
		Eigen::Matrix<double, 6, 1> F_ext;                        // external Forces in x y z 
		Eigen::Matrix<double, 7, 7> mass;                 		// mass matrix [kg]
		Eigen::Matrix<double, 7, 1> coriolis;         // coriolis forces  [Nm]
		Eigen::Matrix<double, 7, 1> q;                 // joint positions  [rad]
		Eigen::Matrix<double, 7, 1> dq;               // joint velocities [rad/s]
		Eigen::Matrix<double, 3, 3> R;
		Eigen::Matrix<double, 3, 1> pos;
		Eigen::Matrix<double, 7, 1> tau_fric;    	// joint friction forces vector
		Eigen::Matrix<double, 7, 1> gravity;    	// gravity vector

		//----------SUBSCRIBERS----------//

		ros::Subscriber sub_inputs_from_matlab;
		void inputs_from_matlab_Callback(const panda_controllers::InputsFromMatlabConstPtr& msg);


		//----------PUBLISHERS----------//

		ros::Publisher pub_info_debug;
		ros::Publisher pub_outputs_to_matlab;

		//----------MESSAGES----------//

		panda_controllers::InfoDebug info_debug_msg;
		panda_controllers::OutputsToMatlab outputs_to_matlab_msg;	
};

}  // namespace franka_softbots
