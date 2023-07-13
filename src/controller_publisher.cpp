#include <vector>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>

#include "panda_controllers/desTrajEE.h"

Eigen::Matrix3d traj_(double t_sym) {
    
    double A = 0.3, a = 0.8;
    double B = 0.3, b = 0.5;
    double d = 0.1;
    double x0 = 0.6, y0 = 0, z0 = 0.5;

    Eigen::Matrix3d trajectory;
    
    trajectory.row(0) << 0+x0, A * std::sin(a * t_sym), B * std::sin(b * t_sym + d);
    trajectory.row(1) << 0, A * a * std::cos(a * t_sym)+y0, B * b * std::cos(b * t_sym + d);
    trajectory.row(2) << 0, -A * a * a * std::sin(a * t_sym), -B * b * b * std::sin(b * t_sym + d) + z0;

    return trajectory;
}

int main(int argc, char** argv)
{
    double time_0;      // Tempo iniziale
    double ros_time;
    double rate_f = 10; // Frequenza di pubblicazione dei messaggi (10 Hz)

    ros::init(argc, argv, "controller_publisher");
 
    ros::NodeHandle nh;
    ros::Publisher des_ee_pub;
    ros::Rate rate(rate_f);

    panda_controllers::desTrajEE desTraj;

    std::vector<double> column;
    std::vector<std::vector<double>> matrix;

    des_ee_pub = nh.advertise<panda_controllers::desTrajEE>("/backstepping_controller/command", 1);
    
    time_0 = (double)ros::Time::now().sec + (double)ros::Time::now().nsec*1e-9;

    Eigen::Matrix3d trajectory_t;
    while (ros::ok())
    {
        desTraj.header.stamp = ros::Time::now();  // Imposta il timestamp corrente        
        ros_time = (double)ros::Time::now().sec + (double)ros::Time::now().nsec*1e-9;

        trajectory_t = traj_(ros_time);
        desTraj.position.x = trajectory_t(0,0);
        desTraj.position.y = trajectory_t(0,1);
        desTraj.position.z = trajectory_t(0,2);
        
        desTraj.velocity.x = trajectory_t(1,0);
        desTraj.velocity.y = trajectory_t(1,1);
        desTraj.velocity.z = trajectory_t(1,2);
        
        desTraj.acceleration.x = trajectory_t(2,0);
        desTraj.acceleration.y = trajectory_t(2,1);
        desTraj.acceleration.z = trajectory_t(2,2);

        des_ee_pub.publish(desTraj);

        rate.sleep();
    }
        
    return 0;
}
