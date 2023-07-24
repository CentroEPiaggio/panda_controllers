#ifndef THUNDERPANDA_H
#define THUNDERPANDA_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "gen_regr_fun.h"

namespace regrob{

    /* Class thunderPanda 
    Useful functions for adaptive control with Slotine-Li regressor
    Usable only for generated code from casadi library! */
    class thunderPanda{

        private:

            /* Number of joints */
            int num_joints;
            /* Joints' variables */
            Eigen::VectorXd q,dq,dqr,ddqr;
            /* Variables that minimize dH from qbar respect qmax-qmin */
            Eigen::VectorXd qbar, qmin, qmax;
            /* Output of generated code */
            Eigen::MatrixXd reg_gen, jac_gen, pinvJac_gen, dotPinvJac_gen, kin_gen, dHdistq_gen;
            /* Compute generated code */
            void computeReg_gen(), computeJac_gen(), computePinvJac_gen(), computeDotPinvJac_gen(), computeKin_gen(), computedHdistq_gen();
            
        public:
            
            /* Empty constructor */
            thunderPanda();
            
            /* Constructor to init variables*/
            thunderPanda(const int);
            
            /* Destructor*/
            ~thunderPanda(){std::cout<<"\nDeleted obj thunderPanda\n";};
            
            /* Init variables */
            void init(const int);
            
            /* Set q, dq, dqr, ddqr, to compute Regressor */
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&);
            
            /* Set q, dq, to compute pseudo-inverse of jacobian and derivate of pseudo-inverse jacobian */
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&);
            
            /* Set q to compute forward kinematic */
            void setArguments(const Eigen::VectorXd&);
            
            /* Set q to compute forward kinematic */
            void setArgsdHdistq(const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&);
            
            /* Get regressor matrix */
            Eigen::MatrixXd getReg_gen(){return reg_gen;};
            
            /* Get jacobian matrix */
            Eigen::MatrixXd getJac_gen(){return jac_gen;};
            
            /* Get pseudo-inverse jacobian matrix */
            Eigen::MatrixXd getPinvJac_gen(){return pinvJac_gen;};
            
            /* Get derivative of pseudo-inverse jacobian matrix */
            Eigen::MatrixXd getDotPinvJac_gen(){return dotPinvJac_gen;};
            
            /* Get regressor matrix */
            Eigen::MatrixXd getKin_gen(){return kin_gen;};
    
            /* Get dH matrix */
            Eigen::MatrixXd getdHdistq_gen(){return dHdistq_gen;};
    
    };

}
#endif