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
#include "gen_regr_fun.h"

namespace regrob{

    #define MU          0.01
    #define MYZERO      0

    class SLregressor: public RegBasic{
        private:
            //int numJoints;
            Eigen::Matrix<double,-1,4> DHTable;
            std::string jointsTypes;
            FrameOffset lab2L0;
            FrameOffset Ln2EE;
            bool dumped;

            Eigen::VectorXd q,dq,dqr,ddqr;
            
            casadi::Function regressor_fun;
            casadi::Function jacobian_fun;
            casadi::Function kinematic_fun;
            
            casadi::SX matYr;

            std::vector<int> nonZeroCols;

            std::vector<casadi::SX> args;
            std::vector<casadi::SX> regressor_res;
            std::vector<casadi::SX> jacobian_res;
            std::vector<casadi::SX> kinematic_res;

            Eigen::MatrixXd out_gen;    // output of code generation

            //void searchNonZero();
            void computeReg();
            void computeJac();
            void computeKin();

            static double mapFunction(const casadi::SXElem& elem);
        
        public:

            SLregressor();
            SLregressor(const int,const Eigen::MatrixXd&,const std::string,FrameOffset&,FrameOffset&,const bool dumped_=true);
            ~SLregressor(){std::cout<<"oggetto SLregressor eliminato\n";}
            
            void init(const int,const Eigen::MatrixXd&,const std::string,FrameOffset&,FrameOffset&,const bool dumped_=true);
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&);
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&);
            void setArguments(const Eigen::VectorXd&);
            //void setArgJac(const Eigen::VectorXd&,const Eigen::VectorXd&);
            //void reduceMinCols();
            
            Eigen::MatrixXd allColumns();
            std::vector<int> get_nonZeroCols(){return nonZeroCols;};
            
            Eigen::Matrix4d kinematic();
            Eigen::MatrixXd jacobian();
            Eigen::MatrixXd pinvJacobian();
            Eigen::MatrixXd dotPinvJacobian();
            //void reduceMinCols();
            //void setDumped(const double dumped_);

            void generate_code(std::string&);
            void computeReg_gen();
            
            Eigen::MatrixXd getRegressor_gen();

    };

}
#endif