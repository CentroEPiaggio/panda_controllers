#include <iostream>
#include <eigen3/Eigen/Dense>
#include "utils/Jacobians.h"


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

extern void get_Ja_proj(const double q[7], double Ja[42]);
extern void get_Ja_dot_proj(const double q[7], const double dq[7], double Ja_dot[42]);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

    double q[7] = {1,1,1,1,1,1,1};
    double dq[7] = {0,0,0,0,0,0,0};
    double ja_array[42];
    double ja_dot_array[42];
    get_Ja_proj(q, ja_array);
    get_Ja_dot_proj(q, dq, ja_dot_array);
    Eigen::Map<Eigen::Matrix<double,6,7,Eigen::RowMajor> > Ja(ja_array);
    Eigen::Map<Eigen::Matrix<double,6,7,Eigen::RowMajor> > Ja_dot(ja_dot_array);
    std::cout << "Jacobiano: " << Ja << std::endl;
    std::cout << "Jacobiano derivato: " << Ja_dot << std::endl;

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
   

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
