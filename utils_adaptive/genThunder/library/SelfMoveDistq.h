#ifndef SELFMOVEDISTQ_H
#define SELFMOVEDISTQ_H

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include "CasadiObj.h"

namespace regrob{
    
    /* Class Forward Kinmeatic */    
    class SelfMoveDistq : public CasadiObj{
        
        private:

            /* Override pure virtual function */
            void init() override{};
            /* Initialize and resize variables and function */
            void initVarsFuns();
            /* Update result of dHdistq casadi function */
            void compute();
            /* Create casadi function */
            void init_casadi_functions();

            /* Symbolic variable for dH to minimize distance from q_bar respect q_max-q_min*/
            casadi::SX _q_, _dq_;
            Eigen::VectorXd _qbar_, _qmin_, _qmax_;
            /* Input of casadi function */
            std::vector<casadi::SX> args;
            /* Casadi function */
            casadi::Function gradDistq_fun, dotGradDistq_fun; 
            /* Output of casadi function */
            std::vector<casadi::SX> gradDistq_res, dotGradDistq_res;

            /* Create dHdistq casadi function */
            void gradDistq(), dotGradDistq();

            /* Variable for dH to minimize distance from q_bar respect q_max-q_min*/
            Eigen::VectorXd q, dq;

        protected:

            /* Number of joints */
            int _numJoints_;

        public:

            /* Empty costructor */
            SelfMoveDistq();
            /* Init variables */
            SelfMoveDistq(const int numJoints, Eigen::VectorXd qmin_, Eigen::VectorXd qmax_, Eigen::VectorXd qbar_);
            /* Init variables */
            void init(const int numJoints, Eigen::VectorXd qmin_, Eigen::VectorXd qmax_, Eigen::VectorXd qbar_);
            /* Destructor */
            ~SelfMoveDistq(){};
            /* Set arguments to update the result of gradient H that minimize the distance of q_ and q_bar respect qmax_-qmin_ */
            void setArguments(const Eigen::VectorXd& q_);
            /* Set arguments to update the result of gradient H that minimize the distance of q_ and q_bar respect qmax_-qmin_ */
            void setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_);
            /* Get forward kinematic matrix */
            Eigen::MatrixXd getGradDistq();
            Eigen::MatrixXd getDotGradDistq();

            /* Generate code */
            void generate_code(std::string&);
            /* Get function name used to generate code in SelfMoveDistq */
            std::vector<std::string> getFunctionsName();
            /* Get casadi function */
            std::vector<casadi::Function> getCasadiFunctions();

    };

}

#endif