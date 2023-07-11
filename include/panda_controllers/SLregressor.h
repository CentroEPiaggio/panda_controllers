#ifndef SLREGRESSOR_
#define SLREGRESSOR_

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>

namespace regrob{
    class SLregressor{
        private:
            int numJoints;
            Eigen::MatrixXd DHTable;
            std::string jointsTypes;
            // frame base;

            Eigen::VectorXd q,dq,dqr,ddqr;
            
            casadi::Function regressor_fun;
            std::vector<int> nonZeroCols;

            std::vector<casadi::DM> args;
            std::vector<casadi::DM> res;

            void searchNonZero();
            void compute();
        public:
            SLregressor(int,const Eigen::MatrixXd&,const std::string);
            ~SLregressor(){std::cout<<"oggetto SLregressor eliminato)\n";}
            void setArguments(const std::vector<Eigen::VectorXd>&);
            Eigen::MatrixXd allColumns();
    };

}
#endif