#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

//#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>

#include "panda_controllers/point.h"
#include "panda_controllers/desTrajEE.h"

#include "utils/ThunderPanda.h"
#include "utils/utils_cartesian.h"

#define NJ 7

typedef Eigen::Vector3d vec3d;

using std::cout;
using std::cin;
using std::endl;

/* variables for message */ ;    
Eigen::Matrix<double, 3, 1> ee_pos_cmd;             // desired command position 
Eigen::Matrix<double, 3, 1> ee_vel_cmd;             // desired command velocity 
Eigen::Matrix<double, 3, 1> ee_acc_cmd;             // desired command acceleration

Eigen::Matrix<double, 3, 3> ee_rot_cmd;             // desired command position
Eigen::Matrix<double, 3, 1> ee_ang_vel_cmd;         // desired command velocity 
Eigen::Matrix<double, 3, 1> ee_ang_acc_cmd;         // desired command acceleration 

/* Obtain end-effector pose */
void desPoseCallback(const panda_controllers::desTrajEE& msg);

/* flag */
bool start = false;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clik_node");
	ros::NodeHandle node_handle;
    double frequency = 1000;
	ros::Rate loop_rate(frequency); // 100 Hz,10 volte più lento del controllore
	
	/* Publisher */
	ros::Publisher pub_des_jointState = node_handle.advertise<sensor_msgs::JointState>("/computed_torque_controller/command", 1);
	
	/* Subscriber */
	ros::Subscriber sub_des_pose = node_handle.subscribe("/backstepping_controller/command", 1, &desPoseCallback);

	/* Message for /computed_torque_controller/command */
	sensor_msgs::JointState clik;
    clik.position.resize(NJ);
    clik.velocity.resize(NJ);
    clik.effort.resize(NJ);

    /* robot kinematic */
    regrob::thunderPanda robot_handle;
    robot_handle.init(NJ);
    Eigen::Matrix<double,4,4> T0EE;
    Eigen::Matrix<double,6,NJ> JacEE;
    Eigen::Matrix<double,NJ,6> pJacEE;
    Eigen::Matrix<double,NJ,6> dot_pJacEE;
    Eigen::Matrix<double,6,6> Lambda;
    std::vector<double> gainLambda(6);

    Eigen::Matrix<double, NJ, 1> qr;
    Eigen::Matrix<double, NJ, 1> qr_old;
    Eigen::Matrix<double, NJ, 1> dot_qr;
    Eigen::Matrix<double, NJ, 1> ddot_qr;

    /* Error and dot error feedback */
    
    Eigen::Matrix<double, 6, 1> error;
    Eigen::Matrix<double, 6, 1> dot_error;

    Eigen::Vector3d ee_position, ee_velocity;
	Eigen::Vector3d ee_omega;
	Eigen::VectorXd ee_vel_cmd_tot(6), ee_acc_cmd_tot(6);
	Eigen::VectorXd tmp_position(6), tmp_velocity(6);
	
  	Eigen::Matrix<double,3,3> ee_rot;
  	Eigen::Matrix<double,3,3> Rs_tilde;
  	Eigen::Matrix<double,3,3> L, dotL;

	Eigen::Matrix<double,6,6> tmp_conversion0, tmp_conversion1, tmp_conversion2;

    qr.setZero();
    qr << 0,-0.7854, 0, -2.3562, 0, 1.5708, 0.7854;
    qr_old.setZero();
    dot_qr.setZero();
    ddot_qr.setZero();

    ee_rot_cmd.setIdentity();
    ee_ang_vel_cmd.setZero();
    ee_ang_acc_cmd.setZero();

    gainLambda = {10,10,10,0.1,0.1,0.1};
    Lambda.setIdentity();
	for(int i=0;i<6;i++){
		Lambda(i,i) = gainLambda[i];
	}

    double period = 1/frequency;
	ros::Time t;

    ros::Duration(1.0).sleep();

	while (ros::ok()){
    
        ros::spinOnce();

        t = ros::Time::now();

        robot_handle.setArguments(qr,dot_qr);
        T0EE = robot_handle.getKin_gen();
        JacEE = robot_handle.getJac_gen();
        pJacEE = robot_handle.getPinvJac_gen();
        dot_pJacEE = robot_handle.getDotPinvJac_gen();

        if(!start){
            ee_rot_cmd = T0EE.block(0,0,3,3);
            start = true;
        }

        /* ======================================== */
        /*          CLICK ALGORITHM                 */
        /* ======================================== */
        /* Compute error translation */
        ee_position = T0EE.col(3).head(3);
        ee_velocity = JacEE.topRows(3)*dot_qr;

        error.head(3) = ee_pos_cmd - ee_position;
        dot_error.head(3) = ee_vel_cmd - ee_velocity;

        /* Compute error orientation */

        ee_rot = T0EE.block(0,0,3,3);
        ee_omega = JacEE.bottomRows(3)*dot_qr;

        Rs_tilde = ee_rot_cmd*ee_rot.transpose();
        L = createL(ee_rot_cmd, ee_rot);
        dotL = createDotL(ee_rot_cmd, ee_rot, ee_ang_vel_cmd, ee_omega);
        
        error.tail(3) = vect(Rs_tilde);
        dot_error.tail(3) = L.transpose()*ee_ang_vel_cmd-L*ee_omega;

        /* Compute reference */

        ee_vel_cmd_tot << ee_vel_cmd, L.transpose()*ee_ang_vel_cmd;
        ee_acc_cmd_tot << ee_acc_cmd, dotL.transpose()*ee_ang_vel_cmd + L.transpose()*ee_ang_acc_cmd;
        
        tmp_position = ee_vel_cmd_tot + Lambda * error;
        tmp_velocity = ee_acc_cmd_tot + Lambda * dot_error;
        
        tmp_conversion0.setIdentity();
        tmp_conversion0.block(3, 3, 3, 3) = L;
        tmp_conversion1.setIdentity();
        tmp_conversion1.block(3, 3, 3, 3) = L.inverse();
        tmp_conversion2.setZero();
        tmp_conversion2.block(3, 3, 3, 3) = -L.inverse() * dotL *L.inverse();

        dot_qr = pJacEE*tmp_conversion1*tmp_position;
        ddot_qr = pJacEE*tmp_conversion1*tmp_velocity + pJacEE*tmp_conversion2*tmp_position +dot_pJacEE*tmp_conversion1*tmp_position;
        qr_old = qr;
        qr = qr_old + period * dot_qr;
        /* ======================================== */
        /*          UPDATE MESSAGE */
        /* ======================================== */
        clik.header.stamp = t;
        for(int i=0;i<NJ;i++){
            clik.position[i] = qr(i);
            clik.velocity[i] = dot_qr(i);
            clik.effort[i] = ddot_qr(i);
        }

        pub_des_jointState.publish(clik);  
		loop_rate.sleep();
	} 

	return 0;
}

void desPoseCallback(const panda_controllers::desTrajEE& msg){

    ee_pos_cmd << msg.position.x, msg.position.y, msg.position.z;
    ee_vel_cmd << msg.velocity.x, msg.velocity.y, msg.velocity.z;
    ee_acc_cmd << msg.acceleration.x, msg.acceleration.y, msg.acceleration.z;
}