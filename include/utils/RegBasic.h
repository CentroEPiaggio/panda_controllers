#ifndef REGLIB_H
#define REGLIB_H

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <stdexcept>

#include "FrameOffset.h"

namespace regrob{

    /* Class RegBasic:
    Basic function for Slotine Li Regressor */    
    class RegBasic {

        private:
            
            /* Variable joint's angles */
            casadi::SX q;
            /* Variable joint's velocities */
            casadi::SX dq;
            /* Variable joint's velocities reference */
            casadi::SX dqr;
            /* Variable joint's accelerationts reference */
            casadi::SX ddqr;

            /* Tensor E for extract tensor of inertia parameters */
            casadi::SXVector E;
            /* Tensor Q for extract CoM positions */
            casadi::SXVector Q;
            /* Matrix to semplify construction of C with symbols of Christoffels */
            casadi::SX dq_sel;

            /* Creation of dq_sel matrix */
            casadi::SX dq_select();
            /* Creation of tensor E */
            casadi::SXVector createE();
            /* Creation of tensor Q */
            casadi::SXVector createQ();

        protected:

            int numJoints;
            
            /* Empty constructor */
            RegBasic();
            
            /* Constructor to init variables */
            RegBasic(const int numJoints_);
            
            /* Destructor */
            ~RegBasic(){std::cout<<"\n RegBasic destruct\n";};
            
            /* Init variables */
            void basic_init(const int);
            
            /* Compute matrix template of transformation from Denavit-Hartenberg parameterization
            rowDHTable format [a alpha d theta], jtsType is "R" or "P" */
            casadi::SX DHTemplate(const Eigen::MatrixXd& rowDHTable, const casadi::SX qi, char jtsType);
            
            /* Compute Forward Kinematic from Denavit-Hartenberg parameterization for each row of DHTable
            DHTable format [a alpha d theta], jtsType is string of "R" and "P" */
            std::tuple<casadi::SXVector,casadi::SXVector> DHFwKin(const Eigen::MatrixXd& DHTable, const std::string& jtsType);
            
            /* Operator hat */
            casadi::SX hat(const casadi::SX& v);
            
            /* Compute Jacobian from transformation of link i-th respect link_0 frame (obtainable from a DH table)
            jtsType: is string of "R" and "P"
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link */
            std::tuple<casadi::SXVector,casadi::SXVector> DHJac(
                const casadi::SXVector& T0i_vec, const std::string& jtsType,FrameOffset& base_frame,FrameOffset& ee_frame);
            
            /* Compute C matrix with Christoffel symbols */
            casadi::SXVector stdCmatrix(const casadi::SX& B);
            
            /* Compute Regressor of Slotine Li
            DHTable format [a alpha d theta]
            jtsType: is string of "R" and "P"
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link */
            casadi::SX SXregressor(
                const Eigen::MatrixXd& DHtable,const std::string& jtsType, FrameOffset& base_frame,FrameOffset& ee_frame);
            
            /* Create casadi function to compute regressor of Slotine Li from matrix of regressor */
            casadi::Function DHReg_fun(const casadi::SX& SX_Yr);
            
            /* Create casadi function to compute jacobian, pseudo-jacobian, derivate of pseudo-jacobian
            DHTable format [a alpha d theta]
            jtsType: is string of "R" and "P"
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link 
            mu_: damp used for pseudo-inverse*/
            casadi::Function DHJac_fun(
                const Eigen::MatrixXd& DHtable, const std::string& jtsType, FrameOffset& base_frame,FrameOffset& ee_frame,const double mu_);
            
            /* Create casadi function to compute forward kinematic
            DHTable format [a alpha d theta]
            jtsType: is string of "R" and "P"
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link */
            casadi::Function DHKin_fun(
                const Eigen::MatrixXd& DHtable, const std::string& jtsType, FrameOffset& base_frame,FrameOffset& ee_frame);

    };
}
#endif