#ifndef THUNDERPANDA_H
#define THUNDERPANDA_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "gen_regr_fun.h"

namespace regrob{

    class thunderPanda{
        private:
            int num_joints;
            Eigen::VectorXd q,dq,dqr,ddqr;
            Eigen::MatrixXd reg_gen;    // output of code generated
            Eigen::MatrixXd jac_gen;    // output of code generated
            Eigen::MatrixXd pinvJac_gen;    // output of code generated
            Eigen::MatrixXd dotPinvJac_gen;    // output of code generated
            Eigen::MatrixXd kin_gen;    // output of code generated

            void computeReg_gen();
            void computeJac_gen();
            void computePinvJac_gen();
            void computeDotPinvJac_gen();
            void computeKin_gen();
            
        public:
            thunderPanda();
            thunderPanda(const int);
            ~thunderPanda(){std::cout<<"\ndelted obj thunderPanda\n";};
            void init(const int);
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&);
            
            Eigen::MatrixXd getReg_gen(){return reg_gen;};
            Eigen::MatrixXd getJac_gen(){return jac_gen;};
            Eigen::MatrixXd getPinvJac_gen(){return pinvJac_gen;};
            Eigen::MatrixXd getDotPinvJac_gen(){return dotPinvJac_gen;};
            Eigen::MatrixXd getKin_gen(){return kin_gen;};
    };

}
#endif