#ifndef SLREGRESSOR_
#define SLREGRESSOR_

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>

#include "utils/myLibReg.h"

namespace regrob{
    class SLregressor{
        private:
            int numJoints;
            Eigen::MatrixXd DHTable;
            std::string jointsTypes;
            frame lab2L0;

            Eigen::VectorXd q,dq,dqr,ddqr;
            
            casadi::Function regressor_fun;
            std::vector<int> nonZeroCols;

            std::vector<casadi::DM> args;
            std::vector<casadi::DM> res;

            void searchNonZero();
            void compute();
        public:
            //SLregressor(const int nj_);
            SLregressor();
            SLregressor(const int,const Eigen::MatrixXd&,const std::string,frame&);
            void init(const int,const Eigen::MatrixXd&,const std::string,frame&);
            ~SLregressor(){std::cout<<"oggetto SLregressor eliminato\n";}
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&);
            Eigen::MatrixXd allColumns();
            std::vector<int> get_nonZeroCols(){return nonZeroCols;};
            void generate_code(std::string&);
    };

}
#endif