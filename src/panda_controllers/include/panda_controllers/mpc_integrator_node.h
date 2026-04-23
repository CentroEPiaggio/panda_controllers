#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

// Msg MPC
#include <panda_controllers/MpcSolution.h>

namespace panda_controllers
{

class MpcIntegratorNode
{
public:
    MpcIntegratorNode();

private:
    // =========================
    // CALLBACK
    // =========================
    void mpcCallback(const panda_controllers::MpcSolutionConstPtr &msg);
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr &msg);
    void controlLoop(const ros::TimerEvent &);

    void publishFilteredStateNow();

    // =========================
    // ROS
    // =========================
    ros::NodeHandle nh;

    ros::Subscriber sub_mpc_sol;
    ros::Subscriber sub_joint_states;

    ros::Publisher pub_cmd;
    ros::Publisher pub_filtered_state;

    ros::Timer timer;

    // =========================
    // Stato integratore
    // =========================
    Eigen::VectorXd q_int_;
    Eigen::VectorXd dq_int_;
    Eigen::VectorXd ddq_int_;
    Eigen::VectorXd jerk_opt_;

    double dt_ctrl_ = 0.001;
    bool has_solution_ = false;

    // =========================
    // Watchdog
    // =========================
    ros::Time last_msg_time_;
    double timeout_ = 0.07;

    // =========================
    // Stato filtrato sensore
    // =========================
    Eigen::VectorXd q_filt_;
    Eigen::VectorXd dq_filt_;
    Eigen::VectorXd ddq_filt_;

    Eigen::VectorXd dq_prev_;
    bool first_joint_msg_ = true;

    // =========================
    // Parametri filtro
    // =========================
    double alpha_q_ = 0.5;
    double alpha_dq_ = 0.2;
    double alpha_ddq_ = 0.05;
};

} // namespace panda_controllers
