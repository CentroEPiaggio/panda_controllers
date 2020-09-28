//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/pdcontroller.h>

namespace panda_controllers
{

bool PdController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{  
    /* Initializing node handle */
  
    this->cvc_nh = node_handle;
   
    std::string arm_id; //checking up the arm id of the robot
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("PdController: Could not get parameter arm_id!");
        return false;
    }

    double kp1, kp2, kp3, kv;

    if (!node_handle.getParam("kp1", kp1) || !node_handle.getParam("kp2", kp2) || !node_handle.getParam("kp3", kp3) || !node_handle.getParam("kv", kv)) {
        ROS_ERROR("PdController: Could not get parameter kpi or kv!");
        return false;
    }

    Kp = Eigen::MatrixXd::Identity(7, 7);
    Kp(1,1) = kp1; Kp(2,2) = kp1; Kp(3,3) = kp1; Kp(4,4) = kp1; Kp(5,5) = kp2; Kp(6,6) = kp2; Kp(7,7) = kp3;
    Kv = kv * Eigen::MatrixXd::Identity(7, 7);
    
    /* Assigning the time */
   
    if (!node_handle.getParam("dt", dt)) {
        ROS_ERROR("PdController: Could not get parameter dt!");
        return false;
    }

    /* Naming each joint */

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR("PdController: Error in parsing joints name!");
        return false;
    }

    /*Get model interface: used to perform calculations using the dynamic model of the robot
      in particular for modelHandle */

    franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM("PdController: Error getting model interface from hardware!");
        return false;
    }

    try {
        model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("PdController: Exception getting model handle from interface: " << ex.what());
        return false;
    }

    /* Get state interface: reads the full robot state, from where we can take q, qdot, tau */

    franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM("PdController: Error getting state interface from hardware");
        return false;
    }

    try {
        state_handle_.reset(new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("PdController: Exception getting state handle from interface: " << ex.what());
        return false;
    }

    /* Getting hardware interface: command joint-level torques and read the joint states */

    hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM("PdController: Error getting effort joint interface from hardware!");
        return false;
    }

    /* Creating handles for each joint */

    for (size_t i = 0; i < 7; ++i) {
        try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("PdController: Exception getting joint handles: " << ex.what());
            return false;
        }
    }

    /* Initialize joint (torque,velocity) limits */
    tau_limit << 87, 87, 87, 87, 12, 12, 12;
    q_dot_limit << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61; 

    /* Start command subscriber */

    this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState> ("command", 1, &PdController::setCommandCB, this);   //it verify with the callback that the command has been received
    this->pub_err_ = node_handle.advertise<sensor_msgs::JointState> ("tracking_error", 1);

    return true;
}

void PdController::starting(const ros::Time& time)
{
    /* Getting Robot State in order to get q_curr and dot_q_curr */

    franka::RobotState robot_state = state_handle_->getRobotState();

    /* Mapping actual joints position and actual joints velocity onto Eigen form */

    q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

    /* Security inizialization */

    command_q_d = q_curr;
    command_dot_q_d = dot_q_curr;

}

void PdController::update(const ros::Time& time, const ros::Duration& period)
{

    franka::RobotState robot_state = state_handle_->getRobotState();
    q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

    // std::cout << "The current q is ";
    // for(int i = 0; i < q_curr.rows(); i++){
    //   std::cout << q_curr(i) << " "; 
    // }

    /* tau_J_d is the desired link-side joint torque sensor signals without gravity */

    tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

//     if (!flag) {         //if the flag is false so dot_q_desired is not given
//         command_dot_q_d = (command_q_d - q_curr) / dt;
//     }

    /* Saturate desired velocity to avoid limits */
    for (int i = 0; i < 7; ++i){
        double ith_des_vel = abs(command_dot_q_d(i)/q_dot_limit(i));
        if( ith_des_vel > 1)
            command_dot_q_d = command_dot_q_d / ith_des_vel;
    }

    err = command_q_d - q_curr;
    dot_err = command_dot_q_d - dot_q_curr;

    // Publish tracking errors as joint states
    sensor_msgs::JointState error_msg;
    std::vector<double> err_vec(err.data(), err.data() + err.rows()*err.cols());
    std::vector<double> dot_err_vec(dot_err.data(), dot_err.data() + dot_err.rows()*dot_err.cols());
    error_msg.header.stamp = ros::Time::now();
    error_msg.position = err_vec;
    error_msg.velocity = dot_err_vec;
    this->pub_err_.publish(error_msg);

    /* Proportional Derivative Controller Law */

    // std::cout << "The err is ";
    // for(int i = 0; i < err.rows(); i++){
    //   std::cout << err(i) << " " << std::endl; 
    // }

    // std::cout << "The command_q_d_old is ";
    // for(int i = 0; i < command_q_d_old.rows(); i++){
    //   std::cout << command_q_d_old(i) << " "; 
    // }

    // std::cout << "dt toSec is " << dt.toSec() << std::endl;

    tau_cmd = Kp * err + Kv * dot_err;

    // std::cout << "The commanded torque before saturation is ";
    // for(int i = 0; i < tau_cmd.rows(); i++){
    //   std::cout << tau_cmd(i) << " " << std::endl; 
    // }

    /* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */

    tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);

    // std::cout << "The commanded torque after saturation is ";
    // for(int i = 0; i < tau_cmd.rows(); i++){
    //   std::cout << tau_cmd(i) << " " << std::endl; 
    // }

    /* Saturate torque to avoid torque limit */
    // for (int i = 0; i < 7; ++i){
    //     double ith_torque_rate = abs(tau_cmd(i)/tau_limit(i));
    //     if( ith_torque_rate > 1)
    //         tau_cmd = tau_cmd / ith_torque_rate;
    // }

    /* Set the command for each joint */
    // std::cout << "Sending command of size " << tau_cmd.rows() << " x " << tau_cmd.cols() << std::endl;
    // std::cout << "The commanded torque is ";
    // for(int i = 0; i < tau_cmd.rows(); i++){
    //   std::cout << tau_cmd(i) << " "; 
    // }
    // int trial;
    // std::cin >> trial;

    for (size_t i = 0; i < 7; ++i) {

        joint_handles_[i].setCommand(tau_cmd(i));

    }

}

void PdController::stopping(const ros::Time& time)
{
    //TO DO
//     Eigen::Matrix<double, 7, 1> tau_stop;
//     tau_stop.setZero();
//
//     /* Set null command for each joint (TODO: Is this necessary?)*/
//           for (size_t i = 0; i < 7; ++i) {
//               joint_handles_[i].setCommand(tau_stop(i));
//           }

}

/* Check for the effort commanded */

Eigen::Matrix<double, 7, 1> PdController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d)
{
    Eigen::Matrix<double, 7, 1> tau_d_saturated {};
    for (size_t i = 0; i < 7; i++) {

        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
    }
    return tau_d_saturated;
}

void PdController::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)      //is the callback of the topic sub_command_ (up).
{
    if ((msg->position).size() != 7 || (msg->position).empty()) {

        ROS_FATAL("Desired position has not dimension 7 or is empty!", (msg->position).size());
    }

    if ((msg->velocity).size() != 7 || (msg->velocity).empty()) {

        ROS_FATAL("Desired velocity has not dimension 7 or is empty!", (msg->velocity).size());
    }

    // TODO: Here we assign acceleration to effort (use trajectory_msgs::JointTrajectoryMessage)
    if ((msg->effort).size() != 7 || (msg->effort).empty()) {

        ROS_FATAL("Desired effort (acceleration) has not dimension 7 or is empty!", (msg->effort).size());
    }

    command_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
    command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->velocity).data());

//     if ((msg->velocity).size() != 7 || (msg->velocity).empty()) {
// 
//         ROS_DEBUG_STREAM("Desired velocity has a wrong dimension or is not given. Velocity of the joints will be estimated.");
//         flag = false;
// 
//     } else {
// 
//         command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->velocity).data());
//         flag = true;
//     }
}

}

PLUGINLIB_EXPORT_CLASS(panda_controllers::PdController, controller_interface::ControllerBase);
