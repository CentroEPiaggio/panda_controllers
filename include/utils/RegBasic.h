#ifndef REGLIB_H
#define REGLIB_H

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <stdexcept>

#include "utils/frameLib.h"

namespace regrob{
    
    class RegBasic {
        private:

            

            double TOLLERANCE = 1e-15; // precision of regressor matrix
    
/*             casadi::SX q = casadi::SX::sym("q", n_j,1);
            casadi::SX dq = casadi::SX::sym("dq", n_j,1);
            casadi::SX dqr = casadi::SX::sym("dqr", n_j,1);
            casadi::SX ddqr = casadi::SX::sym("ddqr", n_j,1); */
            casadi::SX q;
            casadi::SX dq;
            casadi::SX dqr;
            casadi::SX ddqr;


            casadi::SXVector E;
            casadi::SXVector Q;
            casadi::SX dq_sel;

            casadi::SX dq_select();
            casadi::SXVector createE();
            casadi::SXVector createQ();

            //double mapToZero(const double elem);
            
        protected:
            int numJoints;
            RegBasic();
            RegBasic(const int numJoints_);
            void basic_init(const int);
            ~RegBasic(){std::cout<<"\nOggetto RegBasic distrutto\n";};
            // Function to generate matrix template of transformation from Denavit-Hartenberg parameterization
            casadi::SX DHTemplate(const Eigen::MatrixXd& rowDHTable, const casadi::SX qi, char jtsType);
            // Function for Forward Kinematic from Denavit-Hartenberg parameterization
            std::tuple<casadi::SXVector,casadi::SXVector> DHFwKin(const Eigen::MatrixXd& DHTable, const std::string& jtsType);
            // Function hat
            casadi::SX hat(const casadi::SX& v);
            // Function for Jacobian of link i-th from Denavit-Hartenberg parameterization
            std::tuple<casadi::SXVector,casadi::SXVector> DHJac(const casadi::SXVector& T0i_vec, const std::string& jtsType,frame& base_frame);
            // Function to obtain elements to construct C matrix with Christoffel symbols
            casadi::SXVector stdCmatrix(const casadi::SX& B);
            //casadi::Function DHRegressor(const Eigen::MatrixXd& DH_table,const std::string& jType, frame& base_frame);
            casadi::SX SXregressor(const Eigen::MatrixXd& DH_table,const std::string& jType, frame& base_frame);
            casadi::Function DHReg_fun(const casadi::SX& SX_Yr);
            casadi::Function DHJac_fun(const Eigen::MatrixXd& DH_table, const std::string& jtsType, frame& base_frame,const double tol_);
            casadi::Function DHKin_fun(const Eigen::MatrixXd& DH_table, const std::string& jtsType, frame& base_frame);

    };
}
#endif