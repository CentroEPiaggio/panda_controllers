#include <iostream>
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>

//#include <panda_controllers/DesiredProjectTrajectory.h>
//#include "utils/parsing_utilities.h"
#include "ros/ros.h"

#include <sstream>
#include <eigen_conversions/eigen_msg.h>

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "qb_interface/handPos.h"
#include "qb_interface/handRef.h"

using namespace std;

float q_hand;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   exit(signum);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "closeHand");

  ros::NodeHandle node_handle;

  ros::Publisher pub_hand = node_handle.advertise<qb_interface::handRef>("/qb_class/hand_ref", 1000);

  // CREATING THE MESSAGE
  qb_interface::handRef handRef;

  // SET SLEEP TIME TO 1000 ms ---> 1 kHz
  ros::Rate loop_rate(1000);

  signal(SIGINT, signal_callback_handler);


  float close_hand = 10000;
  while (ros::ok())
  {
    ros::spinOnce();
    
      handRef.closure.push_back(close_hand);

      pub_hand.publish(handRef);

    // cout <<t <<endl;
    loop_rate.sleep();

  }
  return 0;
}