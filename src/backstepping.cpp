//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/backstepping.h> //library of the backstepping

#include <utils/get_CoriolisMatrix.h> // to obtain the Coriolis Matrix 

namespace panda_controllers
{

bool BackStepping::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
    this->cvc_nh = node_handle;
  
    std::string arm_id; //checking up the arm id of the robot
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("Backstepping: Could not get parameter arm_id!");
        return false;
    }

    /* Inizializing the kd and lambda gains */

    double kd, kp1, kp2, kp3, lambda;

    if (!node_handle.getParam("kd", kd) || !node_handle.getParam("lambda", lambda)) {
        ROS_ERROR("Backstepping: Kd or lambda parameter couldn't be found!");
        return false;
    }

    if (!node_handle.getParam("kp1", kp1) || !node_handle.getParam("kp2", kp2) || !node_handle.getParam("kp3", kp3)) {
        ROS_ERROR("Backstepping: Kp parameters not found!");
        return false;
    }

    Kd = kd * Eigen::MatrixXd::Identity(7, 7);

    // Proportional gain on possition error (trial: different weight on elbow)
    Kp = Eigen::MatrixXd::Identity(7, 7);
    Kp(1,1) = kp1; Kp(2,2) = kp1; Kp(3,3) = kp1; Kp(4,4) = kp1; Kp(5,5) = kp2; Kp(6,6) = kp2; Kp(7,7) = kp3;

    Lambda = lambda * Eigen::MatrixXd::Identity(7, 7);
    
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

    /* Start command subscriber */

    this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState> ("command", 1, &BackStepping::setCommandCB, this); //it verify with the callback that the command has been received
    this->pub_err_ = node_handle.advertise<sensor_msgs::JointState> ("tracking_error", 1);

    return true;
}

void BackStepping::starting(const ros::Time& time)
{
    /* Getting Robot State in order to get q_curr and dot_q_curr */

    franka::RobotState robot_state = state_handle_->getRobotState();

    /* Mapping actual joints position and actual joints velocity onto Eigen Matrix */

    q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

    /* Getting Mass matrix and Coriolis matrix */

    std::array<double, 49> mass_array = model_handle_->getMass();
    get_CoriolisMatrix(robot_state.q.data(), robot_state.dq.data(), Coriolis_matrix_array);

    /* Mapping Mass matrix and Coriolis matrix onto Eigen form */

    M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
    C = Eigen::Map<Eigen::Matrix<double, 7, 7>>(Coriolis_matrix_array);

    /* Secure Initialization */

    command_q_d = q_curr;
    command_q_d_old = q_curr;

    command_dot_q_d = dot_q_curr;
    command_dot_q_d_old = dot_q_curr;

}

void BackStepping::update(const ros::Time&, const ros::Duration& period)
{
    franka::RobotState robot_state = state_handle_->getRobotState();

    std::array<double, 49> mass_array = model_handle_->getMass();
    M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());

    get_CoriolisMatrix(robot_state.q.data(), robot_state.dq.data(), Coriolis_matrix_array);
    C = Eigen::Map<Eigen::Matrix<double, 7, 7>>(Coriolis_matrix_array);

    /* Actual position and velocity of the joints */

    q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

    /* tau_J_d is the desired link-side joint torque sensor signals without gravity */

    tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

    // if (!flag) { // if the flag is false, desired command velocity must be estimated

    //     command_dot_q_d = (command_q_d - command_q_d_old) / dt;
    // }
    
    /* Saturate desired velocity to avoid limits */
    
    for (int i = 0; i < 7; ++i){
        double ith_des_vel = abs(command_dot_q_d(i)/q_dot_limit(i));
        if( ith_des_vel > 1)
            command_dot_q_d = command_dot_q_d / ith_des_vel;
    }

    // command_dot_dot_q_d = (command_dot_q_d - command_dot_q_d_old) / dt;

    error = command_q_d - q_curr;
    dot_error = command_dot_q_d - dot_q_curr;

    // Publish tracking errors as joint states
    sensor_msgs::JointState error_msg;
    std::vector<double> err_vec(error.data(), error.data() + error.rows()*error.cols());
    std::vector<double> dot_err_vec(dot_error.data(), dot_error.data() + dot_error.rows()*dot_error.cols());
    error_msg.header.stamp = ros::Time::now();
    error_msg.position = err_vec;
    error_msg.velocity = dot_err_vec;
    this->pub_err_.publish(error_msg);

    // std::cout << "The error is ";
    // for(int i = 0; i < error.rows(); i++){
    //   std::cout << error(i) << " " << std::endl; 
    // }

    command_dot_qref = command_dot_q_d + Lambda * error;
    command_dot_dot_qref = command_dot_dot_q_d + Lambda * dot_error;

    /* Velocity Tracking Error */

    s = command_dot_qref - dot_q_curr;

    /* Backstepping Control Law in the joint space */

    tau_cmd = M * command_dot_dot_qref + C * command_dot_qref + Kd * s + Kp * error;

    // std::cout << "Position error: " << error << std::endl;

    /* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */

    tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
    
    /* Saturate torque to avoid torque limit */
    // for (int i = 0; i < 7; ++i){
    //     double ith_torque_rate = abs(tau_cmd(i)/tau_limit(i));
    //     if( ith_torque_rate > 1)
    //         tau_cmd = tau_cmd / ith_torque_rate;
    // }

    /* Set the command for each joint */

    for (size_t i = 0; i < 7; i++) {

        joint_handles_[i].setCommand(tau_cmd[i]);

    }

    /* Saving the last position of the desired position and velocity */
    
    // command_dot_q_d_old = command_dot_q_d;
    // command_q_d_old = command_q_d;

}

void BackStepping::stopping(const ros::Time&)
{
    //TO DO
//     Eigen::Matrix<double, 7, 1> tau_stop;
//     tau_stop.setZero();
//
//     /* Set null command for each joint (TODO: Is this necessary?)*/
//     for (size_t i = 0; i < 7; ++i) {
//         joint_handles_[i].setCommand(tau_stop(i));
//     }
}

/* Check the effort commanded */

Eigen::Matrix<double, 7, 1> BackStepping::saturateTorqueRate(
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

void BackStepping::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)
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
    command_dot_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->effort).data());

    // if ((msg->velocity).size() != 7 || (msg->velocity).empty()) {

    //     ROS_DEBUG_STREAM("Desired velocity has a wrong dimension or is not given. Velocity of the joints will be estimated.");
    //     flag = false;

    // } else {

    //     command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->velocity).data());
    //     flag = true;
    // }
}

}

PLUGINLIB_EXPORT_CLASS(panda_controllers::BackStepping, controller_interface::ControllerBase);
