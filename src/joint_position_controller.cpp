// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_controllers/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace panda_controllers {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }
  sub_desired_joint_ = node_handle.subscribe(
      "/panda_command_joint", 1, &JointPositionController::desiredJointCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  // for (size_t i = 0; i < q_start.size(); i++) {
  //   if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
  //     ROS_ERROR_STREAM(
  //         "JointPositionExampleControllerCP: Robot is not in the expected starting position for "
  //         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
  //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
  //     return false;
  //   }
  // }

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    des_pose_[i] = position_joint_handles_[i].getPosition();
    target_pose_[i] = position_joint_handles_[i].getPosition();
  }
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& /*period*/) {
  // elapsed_time_ += period;
   
  for (size_t i = 0; i < 7; ++i) {
      des_pose_[i] = filter_params_*target_pose_[i]+(1-filter_params_)*des_pose_[i];
      position_joint_handles_[i].setCommand(des_pose_[i]);
}

}  // namespace franka_example_controllers


void JointPositionController::desiredJointCallback(
    const std_msgs::Float64MultiArrayConstPtr& msg) {
            for (int i = 0; i < 7; ++i) {
            target_pose_[i] = msg->data.at(i);
            // std::cout << des_pose_[i] << std::endl;
            }
          }

PLUGINLIB_EXPORT_CLASS(panda_controllers::JointPositionController,
                       controller_interface::ControllerBase)
}