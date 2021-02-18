#include <panda_controllers/project_impedance_controller.h>
#include "utils/Jacobians_ee.h"
#include <cmath>
#include <math.h>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "utils/pseudo_inversion.h"

#define   EE_X  0
#define   EE_Y  0
#define   EE_Z  0

/*
modifiche:
  aumentata massa desiderata a 10 kg in task space
 
da fare:
  provare il debug
  provare ad usare una massa desiderata come diagonale della massa vera
*/


namespace panda_controllers {

extern std::string name_space;

//---------------------------------------------------------------//
//                          INIT                                 //
//--------------------------------------------------------------//

bool ProjectImpedanceController::init(  hardware_interface::RobotHW* robot_hw, 
                                        ros::NodeHandle& node_handle) {
  // Name space extraction for add a prefix to the topic name
  int n = 0;
  // std::string name_space;
  name_space = node_handle.getNamespace();
  n = name_space.find("/", 2);
  name_space = name_space.substr(0,n);

  //------------------------------------------------------------//
  //            INITIALIZE SUBSCRIBERS AND PUBLISHERS           //
  //------------------------------------------------------------//

  // subscribers for impedance_project
  sub_des_traj_proj_ =  node_handle.subscribe(  name_space+"/desired_project_trajectory", 10, 
                                                &ProjectImpedanceController::desiredProjectTrajectoryCallback, this,
                                                ros::TransportHints().reliable().tcpNoDelay());

  sub_des_imp_proj_ =   node_handle.subscribe(  name_space+"/desired_impedance_project", 10, 
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

  //------------------------------------------------------------//
  //              INITIALIZE SERVICE CLIENTS                    //
  //------------------------------------------------------------//

  collBehaviourClient = node_handle.serviceClient<franka_msgs::SetFullCollisionBehavior>( name_space 
                                                                                          + "/franka_control/set_full_collision_behavior");

  jointImpedanceClient = node_handle.serviceClient<franka_msgs::SetJointImpedance>( name_space 
                                                                                    + "/franka_control/set_joint_impedance");

  //------------------------------------------------------------//
  //      INITIALIZE NODE, ROBOT HANDLER AND ROBOT INTERFACE    //
  //------------------------------------------------------------//
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


  //-----------------------------------------------------------//
  //              INITIALIZE VARIABLES                         //
  //-----------------------------------------------------------//

  position_d_.setZero();                    // desired position
  or_des.setZero();                         // desired orientation
  dpose_d_.setZero();                       // desired position velocity
  ddpose_d_.setZero();                      // desired acceleration

  F_ext.setZero();                         // measured external force

  //STIFFNESS AND DAMPING MATRICES 
  cartesian_stiffness_.setIdentity();                                                         
  cartesian_damping_.setIdentity();
  //CARTESIAN MASS MATRIX
  cartesian_mass_.setIdentity();

  // SET MATRICES
  cartesian_mass_ << cartesian_mass_*10; //---------------------------------------------------------------AUMENTATA MASSA 
  cartesian_stiffness_.topLeftCorner(3, 3) << 100*Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3) << 100*Eigen::Matrix3d::Identity();
  // cartesian_stiffness_(1,1) = 100;
  // Damping ratio = 1
  cartesian_damping_.topLeftCorner(3, 3) = 2.0 * sqrt(100)*Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3) = 2.0 * sqrt(100)*Eigen::Matrix3d::Identity();
  // cartesian_damping_(1,1) = 2.0 * sqrt(100);
  
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

//------------------------------------------------------------------//
//                         STARTING                                 //
//------------------------------------------------------------------//

void ProjectImpedanceController::starting(const ros::Time& /*time*/) {
  
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();

   // define R matrix for orientation
  Eigen::Matrix<double, 3, 3> R(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()).topLeftCorner(3,3));

  double phi = atan2(R(1,0),R(0,0));
  double theta = atan2(-R(2,0),sqrt(pow(R(2,1),2) + pow(R(2,2),2)));
  double psi = atan2(R(2,1),R(2,2));

  // orientation // GABICCINI (ZYX) but appended as (x=Z, y=Y, z=X)
  or_des << phi, theta, psi;

  // set nullspace equilibrium configuration to initial q  (q_d_nullspace_ = q_initial;)
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

  //----------------------------------------------------------//
  //                            UPDATE                        //
  //----------------------------------------------------------//

void ProjectImpedanceController::update(  const ros::Time& /*time*/,
                                          const ros::Duration& /*period*/) {

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();         // robot state
  std::array<double, 49> mass_array = model_handle_->getMass();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  
  // define Jacobian and Jacobian_dot for project impedance
  double ja_array[42];                  // define Ja matrix
  double ja_dot_array[42];              // define Ja_dot matrix

 
  //----------------------------------------------------------------------//
  /*                      PUBLISH MATRICES FOR PLANNING                  */
  //----------------------------------------------------------------------//

  // publish mass and jacobian
  robot_state_msg.header.stamp = ros::Time::now();
  std::copy(mass_array.begin(), mass_array.end(), robot_state_msg.mass_matrix.begin());

  pub_robot_state_.publish(robot_state_msg);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 7> > mass(mass_array.data());                 // mass matrix [kg]
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());         // coriolis forces  [Nm]
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());                 // joint positions  [rad]
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());               // joint velocities [rad/s]
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());     // previous cycle commanded torques [Nm]
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));       // ee-base homog. transf. matrix
  Eigen::Vector3d position(transform.translation());                                // ee-base position [m]
  Eigen::Quaterniond orientation(transform.linear());                               // ee-base orientation


  //---------------------------------------------------------------------//
  //                        COMPUTE Ja and Ja_dot                        //
  //---------------------------------------------------------------------//
  
  double q_array[7], dq_array[7];
  for (int i=0; i<7; i++){
    q_array[i] = q(i);
    dq_array[i] = dq(i);
  }

  // Compute Ja and NaN removal
  get_Ja_proj(q_array, EE_X, EE_Y, EE_Z, ja_array);
  // Eigen::Map<Eigen::Matrix<double,6,7,Eigen::RowMajor> > ja(ja_array);
  Eigen::Matrix<double,6,7> ja;

  for (int i=0; i<6; i++){
    for(int j=0; j<7; j++){
      if (std::isnan(ja_array[i*7+j])){
        ja(i,j) = 1;
      }else{
        ja(i,j) = ja_array[i*7+j];
      }
    }
  }

  // Compute Ja_dot and NaN removal
  get_Ja_dot_proj(q_array, dq_array, EE_X, EE_Y, EE_Z, ja_dot_array);
  Eigen::Matrix<double, 6, 7> ja_dot;
  for (int i=0; i<6; i++){
    for(int j=0; j<7; j++){
      if (std::isnan(ja_dot_array[i*7+j])){
        ja_dot(i,j) = 1;
      }else{
        ja_dot(i,j) = ja_dot_array[i*7+j];
      }
    }
  }

  // std::cout << "Jacobian:" << std::endl;
  // std::cout << ja << std::endl;
  // std::cout << "Jacobian Dot:" << std::endl;
  // std::cout << ja_dot << std::endl;

  // define R matrix for orientation
  Eigen::Matrix<double, 3, 3> R(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()).topLeftCorner(3,3));

  // GABICCINI (ZYX)
  double phi = atan2(R(1,0),R(0,0));                          
  double theta = atan2(-R(2,0),sqrt(pow(R(2,1),2) + pow(R(2,2),2)));
  double psi = atan2(R(2,1),R(2,2));

  // orientation // GABICCINI (ZYX) but appended as (x=Z, y=Y, z=X)
  Eigen::Matrix<double, 3, 1> or_proj = {phi, theta, psi};


  //-----------------------------------------------------------------------//
  //                           COMPUTE POSE ERROR                          //
  //-----------------------------------------------------------------------//
  
  Eigen::Matrix<double, 6, 1> error;  // pose error 6x1: position error (3x1) [m] - orientation error in axis-angle repr. (3x1) [rad]
  Eigen::Matrix<double, 6, 1> derror; // pose vel. error 6x1: vel. error (3x1) [m] - orientation vel. error NOT IMPLEMENTED!

  // position error expressed in the base frame
  error.head(3) << position - position_d_;
  error.tail(3) << or_proj - or_des;

  // velocity error
  Eigen::Matrix<double, 6, 1> dpose;
  dpose << ja * dq;
  derror.head(3) << dpose.head(3) - dpose_d_.head(3);
  derror.tail(3) << ja.bottomLeftCorner(3,7)*dq - dpose_d_.tail(3);

  //--------------- WRAP-TO-PI-----------------//

  error(3) = atan2(sin(error(3)),cos(error(3)));
  error(4) = atan2(sin(error(4)),cos(error(4)));
  error(5) = atan2(sin(error(5)),cos(error(5)));

  //------------------------------------------------------------------//
  //                    COMPUTE IMPEDANCE CONTROL                     //
  //------------------------------------------------------------------//

  // allocate variables
  Eigen::VectorXd wrench_task(6), tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling - kinematic pseuoinverse
  Eigen::MatrixXd ja_transpose_pinv;
  pseudoInverse(ja.transpose(), ja_transpose_pinv);

  // added pseudoinverse jacobian
  Eigen::MatrixXd ja_pinv;          // 6x7
  pseudoInverse(ja,ja_pinv);

  // define robot mass in task space
  Eigen::Matrix<double, 6, 6> task_mass;
  task_mass << (ja*mass.inverse()*ja.transpose()).inverse();    // 6x6
  for (int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      if (std::isnan(task_mass(i,j))){
        task_mass(i,j) = 1;
      }
    }
  }

  // SETTING Fext to ZERO!!! REMOVE THIS!
  F_ext.setZero();

  // from matalb: Bx*ddzdes - Bx*inv(Bm)*(Dm*e_dot + Km*e) + (Bx*inv(Bm) - I)*F_ext - Bx*Ja_dot*q_dot;
  // project impedance controller
  wrench_task <<  task_mass*ddpose_d_ 
                  - (task_mass*cartesian_mass_.inverse())*(cartesian_damping_*derror + cartesian_stiffness_*error)
                  + (task_mass*cartesian_mass_.inverse() - Eigen::MatrixXd::Identity(6, 6))*F_ext
                  - task_mass*ja_dot*dq;

  // from matlab: tau = (Ja')*F_tau + S*q_dot + tau_fric + G;
  //final tau in joint space
  tau_task << ja.transpose()*wrench_task 
              + coriolis;

  /*
  // Cartesian PD control with damping ratio = 1 - Cartesian desired Wrench as output
   wrench_task << ( - cartesian_stiffness_ * error               // proportional term
                    - cartesian_damping_ * derror );               // derivative term
  //project in joint space
  tau_task << jacobian.transpose() * wrench_task;
  */

  //null projector
  Eigen::Matrix <double, 6, 7> ja_t_inv;
  ja_t_inv << (ja*mass.inverse()*ja.transpose()).inverse()*ja*mass.inverse();

  Eigen::Matrix <double, 7, 7> N;
  N << Eigen::MatrixXd::Identity(7, 7) - ja.transpose()*ja_t_inv;

  tau_nullspace <<  N * ( nullspace_stiffness_ * (q_d_nullspace_ - q)       // proportional term
                          - (2.0 * sqrt(nullspace_stiffness_)) * dq);       // derivative term

  // Desired torque
  // tau_d << tau_task + tau_nullspace;
  tau_d << tau_task;

  // std::cout << "Commanded torque:" << std::endl;
  // std::cout << tau_d << std::endl;

  // std::cout << "Internal torque:" << std::endl;
  // std::cout << tau_J_d << std::endl;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  // std::cout << "Commanded torque 2:" << std::endl;
  // std::cout << tau_d << std::endl;

  // Saturate torque to avoid torque limit
  for (int i = 0; i < 7; ++i){
    double ith_torque_rate = abs(tau_d(i)/tau_limit(i));
    if( ith_torque_rate > 1)
      tau_d = tau_d / ith_torque_rate;
  }

  std::cout << "Error:" << std::endl;
  std::cout << error << std::endl;

  // std::cout << "dError:" << std::endl;
  // std::cout << derror << std::endl;

  Eigen::Matrix<double,6,1> temp_erroro;
  temp_erroro << 0, 0, 0, error(3), error(4), error(5);
  Eigen::Matrix<double,6,1> temp_errorp;
  temp_errorp << error(0), error(1), error(2), 0, 0, 0;
  std::cout << "Mo:" << std::endl;
  std::cout << ja.transpose()*(task_mass*cartesian_mass_.inverse())*cartesian_stiffness_*temp_erroro << std::endl;
  std::cout << "Mp:" << std::endl;
  std::cout << ja.transpose()*(task_mass*cartesian_mass_.inverse())*cartesian_stiffness_*temp_errorp << std::endl;

  // std::cout << "Wrench:" << std::endl;
  // std::cout << wrench_task << std::endl;

  // std::cout << "Ja:" << std::endl;
  // std::cout << ja << std::endl;

  // std::cout << "Commanded torque after sat:" << std::endl;
  // std::cout << tau_d << std::endl;

  //set arm command torques
  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_d(i));


  //----------------------------------------------------------------------//
  /*                                    PUBLISHER                         */
  //----------------------------------------------------------------------//

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
  postion_endeff.pose.orientation.w = 0.0;
  // GABICCINI (ZYX) but appended as (x=Z, y=Y, z=X)
  postion_endeff.pose.orientation.x = or_proj(0);
  postion_endeff.pose.orientation.y = or_proj(1);
  postion_endeff.pose.orientation.z = or_proj(2);

  pub_endeffector_pose_.publish(postion_endeff);

  ee_pos_msg.pose.position.x = position(0);
  ee_pos_msg.pose.position.y = position(1);
  ee_pos_msg.pose.position.z = position(2);

  pub_ee_pose_.publish(ee_pos_msg);



  // Current impedance
  std_msgs::Float64 impedance_msg;
  impedance_msg.data = std::pow(cartesian_stiffness_.norm(),2) + std::pow(cartesian_damping_.norm(),2);

  pub_impedance_.publish(impedance_msg);

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


//---------------------------------------------------------------//
//                          CALLBACK function                    //
//---------------------------------------------------------------//

//we don't considerer cartesian_damping_target_
void ProjectImpedanceController::desiredImpedanceProjectCallback(
  const panda_controllers::DesiredImpedance::ConstPtr& msg){

  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
      cartesian_stiffness_(i, j) = msg->stiffness_matrix[i*6 + j];
      cartesian_damping_(i, j) = msg->damping_matrix[i*6 + j];
    }
  }
}

// abbiamo creato un nuovo file DesiredProjectTrajectory.msg per includere 
// l'accelerazione e i booleani int e comp che ci servono per il planner 

void ProjectImpedanceController::desiredProjectTrajectoryCallback(
    const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
  
  position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  or_des << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
  dpose_d_ << msg->velocity.position.x, msg->velocity.position.y, msg->velocity.position.z, msg->velocity.orientation.x, msg->velocity.orientation.y, msg->velocity.orientation.z;
  ddpose_d_ << msg->acceleration.position.x, msg->acceleration.position.y, msg->acceleration.position.z, msg->acceleration.orientation.x, msg->acceleration.orientation.y, msg->acceleration.orientation.z;

}

void ProjectImpedanceController::f_ext_Callback(const geometry_msgs::WrenchStampedConstPtr& msg){
  F_ext << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, 0, 0, 0;
}

}  // namespace franka_softbots

PLUGINLIB_EXPORT_CLASS(panda_controllers::ProjectImpedanceController,
                       controller_interface::ControllerBase)
