#ifndef THUNDERPANDA_H
#define THUNDERPANDA_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

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
            /* Output of generated code */
            Eigen::MatrixXd reg_gen, jac_gen, dotJac_gen, pinvJac_gen, dotPinvJac_gen, 
                kin_gen, gradDistq_gen, dotGradDistq_gen, massReg_gen, coriolisReg_gen, gravityReg_gen;
            /* Compute generated code */
            void computeReg_gen(), computeJac_gen(), computeDotJac_gen(), computePinvJac_gen(), 
                computeDotPinvJac_gen(), computeKin_gen(), computeGradDistq_gen(), computeDotGradDistq_gen(),
                computeMassReg_gen(), computeCoriolisReg_gen(), computeGravityReg_gen();
            
            //long long regr_p3[regr_fun_SZ_IW];
            //double regr_p4[regr_fun_SZ_W];

        public:
            
            /* Empty constructor */
            thunderPanda();
            /* Constructor to init variables*/
            thunderPanda(const int);
            /* Destructor*/
            ~thunderPanda(){std::cout<<"thunderPanda destruct\n";};
            
            /* Resize variables */
            void resizeVariables();
            /* Init variables */
            void init(const int);
            /* Set q, dq, dqr, ddqr, to compute Regressor */
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&);
            
            void setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_,const Eigen::VectorXd& dqr_);

            /* Set q, dq, to compute pseudo-inverse of jacobian and derivate of pseudo-inverse jacobian */
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&);
            /* Set q to compute forward kinematic */
            void setArguments(const Eigen::VectorXd&);
            
            /* Get regressor matrix */
            Eigen::MatrixXd getReg_gen(){return reg_gen;};
            /* Get regressor matrix */
            Eigen::MatrixXd getMassReg_gen(){return massReg_gen;};
            /* Get regressor matrix */
            Eigen::MatrixXd getCoriolisReg_gen(){return coriolisReg_gen;};
            /* Get regressor matrix */
            Eigen::MatrixXd getGravityReg_gen(){return gravityReg_gen;};
            /* Get jacobian matrix */
            Eigen::MatrixXd getJac_gen(){return jac_gen;};
            /* Get derivative of jacobian matrix */
            Eigen::MatrixXd getDotJac_gen(){return dotJac_gen;};
            /* Get pseudo-inverse jacobian matrix */
            Eigen::MatrixXd getPinvJac_gen(){return pinvJac_gen;};
            /* Get derivative of pseudo-inverse jacobian matrix */
            Eigen::MatrixXd getDotPinvJac_gen(){return dotPinvJac_gen;};
            /* Get regressor matrix */
            Eigen::MatrixXd getKin_gen(){return kin_gen;};
            /* Get matrix */
            Eigen::MatrixXd getGradDistq_gen(){return gradDistq_gen;};
            /* Get matrix */
            Eigen::MatrixXd getDotGradDist_gen(){return dotGradDistq_gen;};
    };

}
#endif