#ifndef PLANNER_H
#define PLANNER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Auto-generated from cfg/ directory.

#include <memory>
#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <panda_controllers/DesiredProjectTrajectory.h>
#include <panda_controllers/EEpose.h>
#include <panda_controllers/InfoPlanner.h>
// #include <panda_controllers/DesiredTrajectory.h>
#include <panda_controllers/DesiredImpedance.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"



//==========================================================================================//
//                                      CLASS PLANNER                                       //
//==========================================================================================//
// It is the planner in one direction
class planner_class{
    private:
        double ki, kc, F_comp, z_int;
        double k_init;
        int set_F_comp, int_prec, comp_prec, z_int_dir;
        int sign(double x);
    public:
        planner_class();
        int damp;
        void set_k_init(double k);
        double planning(double F_max, double e_max, double F_int_max, double F_ext, double z, double z_des, double dz_des, int inter, int comp);
        void get_info(int axis, double *z_int_, int *z_int_dir_, int *set_F_comp_, double *F_comp_, double *ki_, double *kc_, int *damp_);
};


//==========================================================================================//
//                                      NODE PLANNER                                        //
//==========================================================================================//
// Contains planners for x, y, z and the interpolator 
class planner_node{
    public:
        bool init(ros::NodeHandle& node_handle);
        void update();
        planner_node();

    private:
        // planners
        planner_class planner_x;
        planner_class planner_y;
        planner_class planner_z;
        void interpolator(double kx_f, double ky_f, double kz_f);
        double calc_k(double k, double k_f);        // used by interpolator to ensure stability

        //-----variables-----//
        Eigen::Vector3d F_ext;                      // External Forces in x y z
        Eigen::Vector3d pos_d;                      // desired position
        Eigen::Vector3d dpos_d;                     // desired position velocity
        Eigen::Vector3i interaction;                // interaction flags
        Eigen::Vector3i compensation;               // compensation flags
        Eigen::Vector3d ee_pos;                     // end-effector position
        double K[36];                               // stiffness matrix
        double D[36];                               // damping matrix
        double kx, ky, kz;                          // stiffness values
        ros::Time time_prec;                        // time prec

        //-----subscribers-----//
        ros::Subscriber sub_ee_pose;
        void ee_pose_Callback(const geometry_msgs::PoseStampedConstPtr& msg);

        // trajectory
        ros::Subscriber sub_des_traj_proj_;
        void desiredProjectTrajectoryCallback(const panda_controllers::DesiredProjectTrajectoryConstPtr& msg);

        // external forces
        ros::Subscriber sub_ext_forces;
        void f_ext_Callback(const geometry_msgs::WrenchStampedConstPtr& msg);

        //-----publishers-----//
        ros::Publisher pub_impedance;
        panda_controllers::DesiredImpedance desired_impedance_msg;

        // info_planner //
        ros::Publisher pub_info_planner;
        panda_controllers::InfoPlanner info_planner_msg;
};



#endif 