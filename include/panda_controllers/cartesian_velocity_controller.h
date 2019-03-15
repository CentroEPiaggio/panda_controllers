// Copyright (c) 2019 Centro E. Piaggio
// Use of this source code is governed by license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

// ROS msg includes
#include "geometry_msgs/Twist.h"

namespace panda_controllers {

class CartesianVelocityController : public controller_interface::MultiInterfaceController<franka_hw::FrankaVelocityCartesianInterface, franka_hw::FrankaStateInterface> {
 
  public:
 
  // Controller functions
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  // Command setting callback
  void command(const geometry_msgs::Twist::ConstPtr &msg);

  private:

  // ROS Variables
  ros::Subscriber sub_command_;           // For listening to the command topic

  std::array<double, 6> vel_command;      // The command to be sent to the robot

  std::mutex vel_mutex;                   // A mutual exclusion lock for the vel command

  // The interface and handle
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;

};

}
