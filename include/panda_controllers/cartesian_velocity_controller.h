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
#include <franka_msgs/FrankaState.h>

// Filter includes
#include "filters/filter_chain.h"

// Other includes
#include <boost/scoped_ptr.hpp>

namespace panda_controllers {

class CartesianVelocityController : public controller_interface::MultiInterfaceController<franka_hw::FrankaVelocityCartesianInterface, franka_hw::FrankaStateInterface> {
 
  public:
 
  // Controller functions
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  private:

  // ROS Variables
  ros::NodeHandle cvc_nh;
  ros::Subscriber sub_command_;           // For listening to the command topic
  ros::Subscriber sub_franka_;            // For listening to franka_states for errors

  // Variables for the velocity command
  std::array<double, 6> vel_command;      // The requested command
  std::array<double, 6> filt_command;     // The command to be sent to the robot
  std::mutex vel_mutex;                   // A mutual exclusion lock for the vel command

  // Low pass filter variables
  std::vector<std::shared_ptr<filters::FilterChain<double>>> vel_filter;

  // Time variables
  ros::Time last_command_time;            // Last time at which a command was recieved
  ros::Duration command_timeout;          // Timeout after which command is reset to null

  // Cartesian velocity limits
  double translation_limit;
  double rotation_limit;
  double elbow_limit;

  // The interface and handle
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;

  // Emergency and needed variables
  bool franka_ok = true;
  bool print_once = true;

  // Command setting callback
  void command(const geometry_msgs::Twist::ConstPtr &msg);
  
  // Franka states emergency callback
  void get_franka_states(const franka_msgs::FrankaState::ConstPtr &msg);

  // For checking if there are nans or infs in array
  inline bool check_if_finite(geometry_msgs::Twist twist) {
    std::array<double, 6> tmp_command;
    tmp_command = {{twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z}};
    for (auto it : tmp_command) {
      if (!std::isfinite(it)) {
        return false;
        }
    }
    return true;
  }

};

}
