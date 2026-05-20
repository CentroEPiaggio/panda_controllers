#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <geometry_msgs/TwistStamped.h>

// Msg personalizzati
#include <panda_controllers/MpcSolution.h>
#include <panda_controllers/ObstacleStatus.h>
#include <gazebo_msgs/ModelStates.h>

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
    void pallaCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void controlLoop(const ros::TimerEvent &event);

    void publishFilteredStateNow();

    // =========================
    // ROS
    // =========================
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber sub_mpc_sol;
    ros::Subscriber sub_joint_states;
    ros::Subscriber sub_palla;

    // Publisher
    ros::Publisher pub_cmd;
    ros::Publisher pub_filtered_state;
    ros::Publisher pub_palla_filt;

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
    ros::Time last_joint_time_;
    bool first_joint_msg_ = true;

    // =========================
    // Parametri filtro
    // =========================
    double alpha_q_ = 0.5;
    double alpha_dq_ = 0.2;
    double alpha_ddq_ = 0.05;

    // =========================
    // Stato palla (obstacle)
    // =========================
    Eigen::Vector3d p_palla_filt_;
    Eigen::Vector3d v_palla_filt_;
    Eigen::Vector3d p_palla_prev_;
    ros::Time last_palla_time_;
    bool first_palla_msg_ = true;

    double alpha_p_palla_ = 0.7;
    double alpha_v_palla_ = 0.1;

    // =========================
    // Limiti fisici Franka
    // =========================
    static constexpr double q_min[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    static constexpr double q_max[7] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
    static constexpr double dq_min[7] = {-2.175, -2.175, -2.175, -2.175, -2.61, -2.61, -2.61};
    static constexpr double dq_max[7] = {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
    static constexpr double ddq_min[7] = {-15, -7.5, -10, -12.5, -15, -20, -20};
    static constexpr double ddq_max[7] = {15, 7.5, 10, 12.5, 15, 20, 20};

    // =========================
    // Controllo temporale
    // =========================
    ros::Time last_control_time_;
    bool first_control_ = true;
};

} // namespace panda_controllers