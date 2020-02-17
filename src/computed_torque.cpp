//various library on which we work on
#include <pluginlib/class_list_macros.h>
#include <panda_controllers/computed_torque.h> //library of the computed torque 

namespace panda_controllers
{

bool ComputedTorque::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{ 
    this->cvc_nh = node_handle;
    
    std::string arm_id; //checking up the arm id of the robot
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("Computed Torque: Could not get parameter arm_id!");
        return false;
    }

    /* Inizializing the Kp and Kv gains */

    double kp, kv;

    if (!node_handle.getParam("kp", kp) || !node_handle.getParam("kv", kv)) {
        ROS_ERROR("Computed Torque: One of the parameters kp or kv couldn't be found!");
        return false;
    }

    Kp = kp * Eigen::MatrixXd::Identity(7, 7);
    Kv = kv * Eigen::MatrixXd::Identity(7, 7);

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR("Computed Torque: Error in parsing joints name!");
        return false;
    }

    franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM("Computed Torque: Error getting model interface from hardware!");
        return false;
    }

    try {
        model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("Computed Torque: Exception getting model handle from interface: " << ex.what());
        return false;
    }

    franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM("Computed Torque: Error getting state interface from hardware");
        return false;
    }

    try {
        state_handle_.reset(new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("Computed Torque: Exception getting state handle from interface: " << ex.what());
        return false;
    }

    hardware_interface::EffortJointInterface* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM("Computed Torque: Error getting effort joint interface from hardware!");
        return false;
    }

    for (size_t i = 0; i < 7; ++i) {
        try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("Computed Torque: Exception getting joint handles: " << ex.what());
            return false;
        }
    }

    /*Start command subscriber */

    this->sub_command_ = node_handle.subscribe<sensor_msgs::JointState> ("command", 1, &ComputedTorque::setCommandCB, this);   //it verify with the callback that the command has been received
    return true;
}

void ComputedTorque::starting(const ros::Time& time)
{
    /* Getting Robot State in order to get q_curr and dot_q_curr */

    franka::RobotState robot_state = state_handle_->getRobotState();

    std::array<double, 49> mass_array = model_handle_->getMass();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

    /* Mapping actual joints position, actual joints velocity, Mass matrix and Coriolis vector onto Eigen form  */

    q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

    M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
    C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());

    /* Security Initialization */

    command_q_d = q_curr;
    command_q_d_old = q_curr;

    command_dot_q_d = dot_q_curr;
    command_dot_q_d_old = dot_q_curr;

    /* Defining the NEW gains */

    Kp_apix = M * Kp;
    Kv_apix = M * Kv;
}

void ComputedTorque::update(const ros::Time&, const ros::Duration& period)
{
    franka::RobotState robot_state = state_handle_->getRobotState();

    std::array<double, 49> mass_array = model_handle_->getMass();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

    M = Eigen::Map<Eigen::Matrix<double, 7, 7>>(mass_array.data());
    C = Eigen::Map<Eigen::Matrix<double, 7, 1>>(coriolis_array.data());

    /* Actual position and velocity of the joints */

    q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    dot_q_curr = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

    /* tau_J_d is the desired link-side joint torque sensor signals without gravity */

    tau_J_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());

    if (!flag) { // if the flag is false, desired command velocity must be estimated

        command_dot_q_d = (command_q_d - command_q_d_old) / period.toSec();

    }            // Desired commmand acceleration must be estimated

    command_dot_dot_q_d = (command_dot_q_d - command_dot_q_d_old) / period.toSec();

    /* Computed Torque control law */

    error = command_q_d - q_curr;
    dot_error = command_dot_q_d - dot_q_curr;

    tau_cmd = M * command_dot_dot_q_d + C + Kp_apix * error + Kv_apix * dot_error;  // C->C*dq

    /* Verify the tau_cmd not exceed the desired joint torque value tau_J_d */

    tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);

    /* Set the command for each joint */

    for (size_t i = 0; i < 7; i++) {

        joint_handles_[i].setCommand(tau_cmd[i]);
    }

    /* Saving the last desired commmand position and desired command velocity */

    command_q_d_old = command_q_d;
    command_dot_q_d_old = command_dot_q_d;
}

void ComputedTorque::stopping(const ros::Time&)
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

/* Check for the effort commanded */

Eigen::Matrix<double, 7, 1> ComputedTorque::saturateTorqueRate(
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

void ComputedTorque::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)
{
    if ((msg->position).size() != 7 || (msg->position).empty()) {

        ROS_FATAL("Desired position has not dimension 7 or is empty!", (msg->position).size());
    }

    command_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());

    if ((msg->velocity).size() != 7 || (msg->velocity).empty()) {

        ROS_DEBUG_STREAM("Desired velocity has a wrong dimension or is not given. Velocity of the joints will be estimated.");
        flag = false;

    } else {

        command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->velocity).data());
        flag = true;
    }
}

}

PLUGINLIB_EXPORT_CLASS(panda_controllers::ComputedTorque, controller_interface::ControllerBase);
