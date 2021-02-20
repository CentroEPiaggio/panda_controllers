// riga 80
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <eigen3/Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <franka_msgs/SetFullCollisionBehavior.h>
#include <franka_msgs/SetJointImpedance.h>

#include <panda_controllers/DesiredProjectTrajectory.h>
#include <panda_controllers/EEpose.h>
#include <panda_controllers/InfoDebug.h>
// #include <panda_controllers/DesiredTrajectory.h>
#include <panda_controllers/DesiredImpedance.h>
#include <panda_controllers/RobotState.h>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

namespace panda_controllers {

//==========================================================================================//
//                                    CLASS CONTROLLER                                      //
//==========================================================================================//
class ProjectImpedanceController : public controller_interface::MultiInterfaceController
												<franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 	public:
		bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
		void starting(const ros::Time&) override;
		void update(const ros::Time&, const ros::Duration& period) override;
		franka_msgs::SetFullCollisionBehavior collBehaviourSrvMsg;
		ros::ServiceClient collBehaviourClient;
		franka_msgs::SetJointImpedance jointImpedanceSrvMsg;
		ros::ServiceClient jointImpedanceClient;

 	private:
		//----------FRANKA STUFFS----------//
		Eigen::Matrix<double, 7, 1> saturateTorqueRate(
			const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
			const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

		std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
		std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
		std::vector<hardware_interface::JointHandle> joint_handles_;


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
		

		//----------SUBSCRIBERS----------//
		ros::Subscriber sub_des_traj_proj_;
		void desiredProjectTrajectoryCallback(const panda_controllers::DesiredProjectTrajectoryConstPtr& msg);

		ros::Subscriber sub_des_imp_proj_;
		void desiredImpedanceProjectCallback(const panda_controllers::DesiredImpedance::ConstPtr& msg);

		// external forces
		ros::Subscriber sub_ext_forces;
		void f_ext_Callback(const geometry_msgs::WrenchStampedConstPtr& msg);


		//----------PUBLISHERS----------//
		ros::Publisher pub_pos_error;
		ros::Publisher pub_cmd_force;
		ros::Publisher pub_endeffector_pose_;
		ros::Publisher pub_ee_pose_;
		ros::Publisher pub_robot_state_;
		ros::Publisher pub_impedance_;
		ros::Publisher pub_info_debug;
		

		//----------MESSAGES----------//
		geometry_msgs::TwistStamped   pos_error_msg;
		geometry_msgs::WrenchStamped  force_cmd_msg;  
		panda_controllers::RobotState robot_state_msg;
		panda_controllers::EEpose ee_pos_msg;
		panda_controllers::InfoDebug info_debug_msg;	
};

}  // namespace franka_softbots
