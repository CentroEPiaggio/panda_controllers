// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_controllers/project_impedance_controller.h>
#include "utils/get_Ja_dot.h"
#include <cmath>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "utils/pseudo_inversion.h"

namespace panda_controllers {

// added "extern" 
// puo' dare noia utilizzare la stessa stringa per più controlli.cpp? 
  extern std::string name_space;


bool ProjectImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  
  // Name space extraction for add a prefix to the topic name
  int n = 0;
  // std::string name_space;
  name_space = node_handle.getNamespace();
  n = name_space.find("/", 2);
  name_space = name_space.substr(0,n);


  /*--------------------------------------------------INITIALIZE SUBSCRIBERS AND PUBLISHERS*/

  // subscribers for impedance_project
  sub_des_traj_proj_ = node_handle.subscribe(
      name_space+"/desired_trajectory_project", 10, &ProjectImpedanceController::desiredProjectTrajectoryCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_des_imp_proj_ = node_handle.subscribe(
      name_space+"/desired_impedance_project", 10, &ProjectImpedanceController::desiredImpedanceProjectCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
/*
  sub_des_traj_ = node_handle.subscribe(
      name_space+"/desired_trajectory", 10, &ProjectImpedanceController::desiredTrajectoryCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_desired_stiffness_matrix_ = node_handle.subscribe(
      name_space+"/desired_impedance", 1, &ProjectImpedanceController::desiredImpedance_Callback, this,
      ros::TransportHints().reliable().tcpNoDelay());
*/
  pub_pos_error    = node_handle.advertise<geometry_msgs::TwistStamped>(name_space+"/pos_error", 1);
  pub_cmd_force   = node_handle.advertise<geometry_msgs::WrenchStamped>(name_space+"/cmd_force", 1);
  pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>(name_space+"/franka_ee_pose", 1);
  pub_robot_state_ = node_handle.advertise<panda_controllers::RobotState>(name_space+"/robot_state", 1);
  pub_impedance_ = node_handle.advertise<std_msgs::Float64>(name_space+"/current_impedance", 1);

  /*--------------------------------------------------INITIALIZE SERVICE CLIENTS*/
  collBehaviourClient = node_handle.serviceClient<franka_msgs::SetFullCollisionBehavior>(
      name_space + "/franka_control/set_full_collision_behavior");
  jointImpedanceClient = node_handle.serviceClient<franka_msgs::SetJointImpedance>(
      name_space + "/franka_control/set_joint_impedance");

  /*--------------------------------------------------INITIALIZE NODE, ROBOT HANDLER AND ROBOT INTERFACE*/
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

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ProjectImpedanceController: Error getting model interface from hardware");
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

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ProjectImpedanceController: Error getting state interface from hardware");
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

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ProjectImpedanceController: Error getting effort joint interface from hardware");
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



  /*-----------------------------------INITIALIZE VARIABLES */

  position_d_.setZero();                                   // desired position
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;           // desired orientation
  dposition_d_.setZero();                                  // desired position velocity
  ddposition_d_.setZero();                                 // desired acceleration

  F_ext_.setZero();

  //STIFFNESS AND DAMPING MATRICES
  cartesian_stiffness_.setIdentity();                                                         
  // cartesian_stiffness_target_.setIdentity();
  cartesian_damping_.setIdentity();
  // cartesian_damping_target_.setIdentity();
  cartesian_mass_.setIdentity();

  cartesian_stiffness_ << 200 * Eigen::Matrix3d::Identity();
  cartesian_damping_ << 2.0 * sqrt(200)*Eigen::Matrix3d::Identity();
  cartesian_mass_ << 1.0 * Eigen::Matrix3d::Identity();

  // cartesian_stiffness_.topLeftCorner(3, 3) << 200*Eigen::Matrix3d::Identity();
  // cartesian_stiffness_.bottomRightCorner(3, 3) << 20*Eigen::Matrix3d::Identity();
  // cartesian_stiffness_target_ = cartesian_stiffness_;
  // Damping ratio = 1
  // cartesian_damping_.topLeftCorner(3, 3) = 2.0 * sqrt(200)*Eigen::Matrix3d::Identity();
  // cartesian_damping_.bottomRightCorner(3, 3) = 2.0 * sqrt(20)*Eigen::Matrix3d::Identity();
  // cartesian_damping_target_ = cartesian_damping_;
  
  tau_limit << 87, 87, 87, 87, 12, 12, 12;  //joint torques limit vector

  // Collision behaviours limits
  // collBehaviourSrvMsg.request.lower_torque_thresholds_acceleration = {2000, 2000, 2000, 2000, 2000, 2000, 2000};
  // collBehaviourSrvMsg.request.upper_torque_thresholds_acceleration = {2000, 2000, 2000, 2000, 2000, 2000, 2000};
  // collBehaviourSrvMsg.request.lower_torque_thresholds_nominal = {2000, 2000, 2000, 2000, 2000, 2000, 2000};
  // collBehaviourSrvMsg.request.upper_torque_thresholds_nominal = {2000, 2000, 2000, 2000, 2000, 2000, 2000};
  // collBehaviourSrvMsg.request.lower_force_thresholds_acceleration = {2000, 2000, 2000, 2000, 2000, 2000};
  // collBehaviourSrvMsg.request.upper_force_thresholds_acceleration = {2000, 2000, 2000, 2000, 2000, 2000};
  // collBehaviourSrvMsg.request.lower_force_thresholds_nominal = {2000, 2000, 2000, 2000, 2000, 2000};
  // collBehaviourSrvMsg.request.upper_force_thresholds_nominal = {2000, 2000, 2000, 2000, 2000, 2000};

  collBehaviourSrvMsg.request.lower_torque_thresholds_acceleration = {100, 100, 100, 100, 100, 100, 100};
  collBehaviourSrvMsg.request.upper_torque_thresholds_acceleration = {100, 100, 100, 100, 100, 100, 100};
  collBehaviourSrvMsg.request.lower_torque_thresholds_nominal = {100, 100, 100, 100, 100, 100, 100};
  collBehaviourSrvMsg.request.upper_torque_thresholds_nominal = {100, 100, 100, 100, 100, 100, 100};
  collBehaviourSrvMsg.request.lower_force_thresholds_acceleration = {100, 100, 100, 100, 100, 100};
  collBehaviourSrvMsg.request.upper_force_thresholds_acceleration = {100, 100, 100, 100, 100, 100};
  collBehaviourSrvMsg.request.lower_force_thresholds_nominal = {100, 100, 100, 100, 100, 100};
  collBehaviourSrvMsg.request.upper_force_thresholds_nominal = {100, 100, 100, 100, 100, 100};

  // Joint impedance
  jointImpedanceSrvMsg.request.joint_stiffness = {3000, 3000, 3000, 3000, 3000, 2000, 100};

  return true;
}

void ProjectImpedanceController::starting(const ros::Time& /*time*/) {
  
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;

  // set collision behaviour
  collBehaviourClient.call(collBehaviourSrvMsg);

  // set joint impedance
  jointImpedanceClient.call(jointImpedanceSrvMsg);

}

  /*-----------------------------------UPDATE */

void ProjectImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();                  // robot state
  std::array<double, 49> mass_array = model_handle_->getMass();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  // std::array<double, 42> jacobian_array = model_handle_->getBodyJacobian(franka::Frame::kEndEffector);
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  
  double ja_dot_array[21];              // define Ja_dot matrix

  /*-------------------------------------------------------PUBLISH MATRICES FOR PLANNING*/

  // publish mass and jacobian
  robot_state_msg.header.stamp = ros::Time::now();
  std::copy(mass_array.begin(), mass_array.end(), robot_state_msg.mass_matrix.begin());
  std::copy(jacobian_array.begin(), jacobian_array.end(), robot_state_msg.jacobian_matrix.begin());

  pub_robot_state_.publish(robot_state_msg);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 7> > mass(mass_array.data());                 // mass matrix [kg]
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());         // coriolis forces  [Nm]
  Eigen::Map<Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());           // gravity forces [N]
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());         // jacobian
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());                 // joint positions  [rad]
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());               // joint velocities [rad/s]
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());     // previous cycle commanded torques [Nm]
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));       // ee-base homog. transf. matrix
  Eigen::Vector3d position(transform.translation());                                // ee-base position [m]
  Eigen::Quaterniond orientation(transform.linear());                               // ee-base orientation



  /*--------------------------------------------COMPUTE Ja_dot */
  

 // Compute Ja_dot
  double q_array[7], dq_array[7];
  for (int i=0; i<7; i++){
    q_array[i] = q.coeff(i,1);
    dq_array[i] = dq.coeff(i,1);
  }

  get_Ja_dot(q_array, dq_array,ja_dot_array);

  Eigen::Map<Eigen::Matrix<double, 3, 7> > ja_dot(ja_dot_array);


  /*--------------------------------------------COMPUTE POSE ERROR */
  


  Eigen::Matrix<double, 6, 1> error;  // pose error 6x1: position error (3x1) [m] - orientation error in axis-angle repr. (3x1) [rad]
  Eigen::Matrix<double, 6, 1> derror; // pose vel. error 6x1: vel. error (3x1) [m] - orientation vel. error NOT IMPLEMENTED!

  // position error expressed in the base frame
  error.head(3) << position - position_d_;
  Eigen::Matrix<double, 6, 1> dposition = jacobian * dq;
  derror.head(3) << dposition.head(3) - dposition_d_;

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
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  
  // transposition of the linear and angular errors in the end-effector
  // error.head(3) = transform.linear().transpose()*error.head(3);
  // error.tail(3) = transform.linear().transpose()*error.tail(3);

  // TODO: implement velocity orientation error
  derror.tail(3) = dposition.tail(3);


  /*------------------------------------------------------COMPUTE CONTROL*/

  // allocate variables
  Eigen::VectorXd wrench_task(3), tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling - kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // added pseudoinverse jacobian
  Eigen::MatrixXd jacobian_pinv;          // 7x3
  pseudoInverse(jacobian.topLeftCorner(3,7),jacobian_pinv);

  // define robot mass in task space
  Eigen::Matrix<double, 3, 3> task_mass;
  task_mass << jacobian_transpose_pinv*mass*jacobian_pinv;    // 3x3

// from matalb: Bx*ddzdes - Bx*inv(Bm)*(Dm*e_dot + Km*e) + (Bx*inv(Bm) - I)*F_ext - Bx*Ja_dot*q_dot;
// project impedance controller
  wrench_task << task_mass*ddposition_d_ 
                  - (task_mass*cartesian_mass_.inverse())*(cartesian_damping_*derror.head(3) + cartesian_stiffness_*error.head(3))
                  + (task_mass*cartesian_mass_.inverse() - Eigen::MatrixXd::Identity(3, 3))*F_ext_
                  - task_mass*ja_dot*dq;

// from matlab: tau = (Ja')*F_tau + S*q_dot + tau_fric + G;
 //final tau in joint space
tau_task << jacobian.transpose()*wrench_task + coriolis + gravity;

/*
  // Cartesian PD control with damping ratio = 1 - Cartesian desired Wrench as output
   wrench_task << ( - cartesian_stiffness_ * error               // proportional term
                    - cartesian_damping_ * derror );               // derivative term
  //project in joint space
  tau_task << jacobian.transpose() * wrench_task;
  */
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *  //null projection
                    ( nullspace_stiffness_ * (q_d_nullspace_ - q)       // proportional term
                    - (2.0 * sqrt(nullspace_stiffness_)) * dq);         // derivative term

  // Desired torque
  tau_d << tau_task + tau_nullspace;
  // tau_d << tau_task + coriolis;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  // Saturate torque to avoid torque limit
  for (int i = 0; i < 7; ++i){
    double ith_torque_rate = abs(tau_d(i)/tau_limit(i));
    if( ith_torque_rate > 1)
      tau_d = tau_d / ith_torque_rate;
  }

  //set arm command torques
  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_d(i));


  /*-------------------------------------------------------PUBLISH*/

  // position error
  pos_error_msg.header.stamp = ros::Time::now();
  pos_error_msg.twist.linear.x = -error(0);
  pos_error_msg.twist.linear.y = -error(1);
  pos_error_msg.twist.linear.z = -error(2);
  pos_error_msg.twist.angular.x = -error(3);
  pos_error_msg.twist.angular.y = -error(4);
  pos_error_msg.twist.angular.z = -error(5);

  pub_pos_error.publish(pos_error_msg);

  // commanded wrench
  force_cmd_msg.wrench.force.x = wrench_task(0);
  force_cmd_msg.wrench.force.y = wrench_task(1);
  force_cmd_msg.wrench.force.z = wrench_task(2);
  force_cmd_msg.wrench.torque.x = wrench_task(3);
  force_cmd_msg.wrench.torque.y = wrench_task(4);
  force_cmd_msg.wrench.torque.z = wrench_task(5);

  pub_cmd_force.publish(force_cmd_msg);

  // End effector position
  geometry_msgs::PoseStamped postion_endeff;
  postion_endeff.pose.position.x = position.x();
  postion_endeff.pose.position.y = position.y();
  postion_endeff.pose.position.z = position.z();
  postion_endeff.pose.orientation.w = orientation.w();
  postion_endeff.pose.orientation.x = orientation.x();
  postion_endeff.pose.orientation.y = orientation.y();
  postion_endeff.pose.orientation.z = orientation.z();

  pub_endeffector_pose_.publish(postion_endeff);

  // Current impedance
  std_msgs::Float64 impedance_msg;
  impedance_msg.data = std::pow(cartesian_stiffness_.norm(),2) + std::pow(cartesian_damping_.norm(),2);

  pub_impedance_.publish(impedance_msg);

  /*---------------------------------------------------UPDATE desired cartesian
  stiffness and damping matrices target by filtering*/
  /*
  cartesian_stiffness_ =  filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =    filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  */

}



Eigen::Matrix<double, 7, 1> ProjectImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated;
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

/*---------------------------------------------------CALLBACK function
*/
//we don't considerer cartesian_damping_target_
void ProjectImpedanceController::desiredImpedanceProjectCallback(
    const panda_controllers::DesiredImpedance::ConstPtr& msg){
  
  for (int i=0; i<3; i++){
     for (int j=0; j<3; j++){
        // sub-matrix 3x3 from stiffness matrix
        cartesian_stiffness_(i, j) = msg->stiffness_matrix[i*6 + j];
     }
  }

  if (var_damp) {
    for (int i=0; i<3; i++){
      for (int j=0; j<3; j++){
          // sub-matrix 3x3 from damping matrix
          cartesian_damping_(i, j) = msg->damping_matrix[i*6 + j];
      }
    }
  } else {
    cartesian_damping_.Identity();
    for (int i=0; i<3; i++){
      for (int j=0; j<3; j++){
          // sub-matrix 3x3 from damping matrix 
          cartesian_damping_(i, j) = msg->damping_matrix[i*6 + j];
      }
    }
  }
}

/*
void ProjectImpedanceController::desiredImpedance_Callback(
    const panda_controllers::DesiredImpedance::ConstPtr& msg){
  
  for (int i=0; i<36; i++)
    cartesian_stiffness_(i) = msg->stiffness_matrix[i];
    // cartesian_stiffness_target_(i) = msg->stiffness_matrix[i];

  if (var_damp) {
    for (int i=0; i<36; i++)
      cartesian_damping_(i) = msg->damping_matrix[i];
      // cartesian_damping_target_(i) = msg->damping_matrix[i];
  } else {
    cartesian_damping_.Identity();
    // cartesian_damping_target_.Identity();
    for (int i=0; i<6; i++)
      cartesian_damping_(i,i) = 2.0 * sqrt(cartesian_stiffness_(i,i));
      // cartesian_damping_target_(i,i) = 2.0 * sqrt(cartesian_stiffness_target_(i,i));
  }
  // std::cout << "Cartesian Stiffness: \n" << cartesian_stiffness_target_ << std::endl;
  // std::cout << "Cartesian Damping: \n" << cartesian_damping_target_ << std::endl;

}
*/


// abbiamo creato un nuovo file DesiredProjectTrajectory.msg per includere 
// l'accelerazione e i booleani int e comp che ci serrvono per il planner 

void ProjectImpedanceController::desiredProjectTrajectoryCallback(
    const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
  
  position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  dposition_d_ << msg->velocity.position.x, msg->velocity.position.y, msg->velocity.position.z;
  ddposition_d_ << msg->acceleration.position.x, msg->acceleration.position.y, msg->acceleration.position.z;

}

/*
void ProjectImpedanceController::desiredTrajectoryCallback(
    const panda_controllers::DesiredTrajectoryConstPtr& msg) {
  
  position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  dposition_d_ << msg->velocity.position.x, msg->velocity.position.y, msg->velocity.position.z;

  Eigen::Quaterniond last_orientation_d_target(orientation_d_);
  orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;

  if (last_orientation_d_target.coeffs().dot(orientation_d_.coeffs()) < 0.0)
    orientation_d_.coeffs() << -orientation_d_.coeffs();

}
*/



}  // namespace franka_softbots

PLUGINLIB_EXPORT_CLASS(panda_controllers::ProjectImpedanceController,
                       controller_interface::ControllerBase)
