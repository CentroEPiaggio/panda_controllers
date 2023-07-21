#ifndef SLREGRESSOR_H
#define SLREGRESSOR_H

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>

#include "RegBasic.h"

namespace regrob{

    #define MU          0.01
    #define MYZERO      0

    /* Class SLregressor:
    Useful functions for adaptive control with Slotine-Li regressor */
    class SLregressor: public RegBasic{
        private:
            
            /* Denavit-Hartenberg table */
            Eigen::Matrix<double,-1,4> DHTable;
            /* String for joints type*/
            std::string jointsTypes;
            /* Frame offset between world-frame and link 0*/
            FrameOffset lab2L0;
            /* Frame offset between end-effector and last link */
            FrameOffset Ln2EE;
            /* Flag to use dumped pseudo-inverse of jacobian */
            bool dumped;
            /* Variable for joints */
            Eigen::VectorXd q,dq,dqr,ddqr;
            
            /* Casadi function */
            casadi::Function regressor_fun, jacobian_fun, pinvJacobian_fun, dotPinvJacobian_fun, kinematic_fun;
            /* Input of casadi function */
            std::vector<casadi::SX> args;
            /* Output of casadi function */
            std::vector<casadi::SX> regressor_res, jacobian_res, pinvJacobian_res, dotPinvJacobian_res, kinematic_res;
            
            /* Matrix for regressor */
            casadi::SX matYr;

            /* Update regressor result */
            void computeReg();
            /* Update jacobian, pseudo-inverse jacobian, derivate of pseudo-inverse of jacobian result */
            void computeJac();
            /* Update forward kinematic result */
            void computeKin();

            /* Function to transform casadi element to double */
            static double mapFunction(const casadi::SXElem& elem);
        
        public:

            /* Empty costructor */
            SLregressor();

            /* Constructor to init variables
            nj_: number of joints
            DHTable_: Denavit-Hartenberg table format [a alpha d theta]
            jtsType_: is string of "R" and "P"
            base_: is used to set transformation between link0 and world_frame
            ee_: is used to set transformation between end-effector and last link 
            dumped_: damp used for pseudo-inverse*/
            SLregressor(const int,const Eigen::MatrixXd&,const std::string,FrameOffset&,FrameOffset&,const bool dumped_=true);
            
            /* Destructor */
            ~SLregressor(){std::cout<<"oggetto SLregressor eliminato\n";}
            
            /* Init variables
            nj_: number of joints
            DHTable_: Denavit-Hartenberg table format [a alpha d theta]
            jtsType_: is string of "R" and "P"
            base_: is used to set transformation between link0 and world_frame
            ee_: is used to set transformation between end-effector and last link 
            dumped_: damp used for pseudo-inverse*/
            void init(const int,const Eigen::MatrixXd&,const std::string,FrameOffset&,FrameOffset&,const bool dumped_=true);

            /* Set q, dq, dqr, ddqr, to compute Regressor */
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&);

            /* Set q, dq, to compute jacobian, pseudo-inverse jacobian, 
            derivate of pseudo-inverse jacobian, forward kinematic */
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&);

            /* Set q to compute forward kinematic */
            void setArguments(const Eigen::VectorXd&);
            
            /* Get regressor matrix */
            Eigen::MatrixXd allColumns();
            
            /* Get forward kinematic matrix */
            Eigen::Matrix4d kinematic();
            
            /* Get forward jacobian matrix */
            Eigen::MatrixXd jacobian();
            
            /* Get forward pseudo-inverse of jacobian matrix */
            Eigen::MatrixXd pinvJacobian();

            /* Get forward derivate pseudo-inverse of jacobian matrix */
            Eigen::MatrixXd dotPinvJacobian();

            //void reduceMinCols();

            /* Generate code in cpp to lose dependencies from casadi library. 
            The generated code is to evaluate regressor, jacobian, pseudo-inverse jacobian,
            derivate of pseudo-inverse jacobian, forward kinematic */
            void generate_code(std::string&);

    };

}
#endif