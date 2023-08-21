#ifndef ROBREG_H
#define ROBREG_H

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include "RobKinBasic.h"
#include "FrameOffset.h"

namespace regrob{
    
    /* Class Forward Kinmeatic */    
    class RobReg : public RobKinBasic{
        
        private:

            /* Override pure virtual function */
            virtual void init() override{};
            /* Initialize and resize variables and function */
            virtual void initVarsFuns();
            /* Set arguments to update the result of forward kinematic and jacobian */
            virtual void setArguments(const Eigen::VectorXd&) override{};
            /* Update result of kinematic casadi function */
            virtual void compute();
            /* Create casadi function */
            virtual void init_casadi_functions();

            /* Input of casadi function */
            std::vector<casadi::SX> args;

            /* Matrix SX of regressor */
            casadi::SX SX_Yr, SX_mass, SX_coriolis, SX_gravity;
            /* Variable for joints to set arguments*/
            Eigen::VectorXd q, dq, dqr, ddqr;

        protected:

            /* Variable for joints */
            casadi::SX _dq_, _dqr_, _ddqr_;

            /* Create regressor casadi function */
            void Regressor();
            /* Create regressor casadi function */
            void massReg();
            /* Create regressor casadi function */
            void coriolisReg();
            /* Create regressor casadi function */
            void gravityReg();

            /* Output of casadi function */
            std::vector<casadi::SX> regressor_res, massReg_res, coriolisReg_res, gravityReg_res;
            /* Casadi function */
            casadi::Function regressor_fun, massReg_fun, coriolisReg_fun, gravityReg_fun;

            /* Creation of dq_sel matrix */
            casadi::SX dq_select(const casadi::SX& dq_);
            /* Creation of tensor E */
            casadi::SXVector createE();
            /* Creation of tensor Q */
            casadi::SXVector createQ();

            /* Compute C matrix with Christoffel symbols */
            casadi::SXVector stdCmatrix(const casadi::SX& B, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_);
            /* Compute Regressor of Slotine Li from Denavit-Hartenberg parameterization*/
            casadi::SXVector SXregressor(
                const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dqr_, const casadi::SX& ddqr_, 
                const std::string jointsType_, const Eigen::MatrixXd& DHtable_, FrameOffset& base_frame,FrameOffset& ee_frame);

        public:

            /* Empty costructor */
            RobReg();
            /* Init variables
            numJoints: number of joints
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            jtsType: is string of "R" and "P"
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link */
            RobReg(const int numJoints,const std::string jointsType,const Eigen::MatrixXd& DHtable,
                FrameOffset& base_frame, FrameOffset& ee_frame);
            /* Init variables
            numJoints: number of joints
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            jtsType: is string of "R" and "P"
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link */
            virtual void init(const int numJoints,const std::string jointsType,const Eigen::MatrixXd& DHtable,
                FrameOffset& base_frame, FrameOffset& ee_frame);
            /* Destructor */
            ~RobReg(){};
            
            virtual void setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_, const Eigen::VectorXd& dqr_, const Eigen::VectorXd& ddqr_);

            /* Get regressor matrix */
            Eigen::MatrixXd getRegressor();
            /* Get regressor matrix */
            Eigen::MatrixXd getMassReg();
            /* Get regressor matrix */
            Eigen::MatrixXd getCoriolisReg();
            /* Get regressor matrix */
            Eigen::MatrixXd getGravityReg();
            /* Generate code for regressor */
            virtual void generate_code(std::string&);
            /* Get function name used to generate code in RobReg */
            virtual std::vector<std::string> getFunctionsName();
            /* Get casadi function */
            virtual std::vector<casadi::Function> getCasadiFunctions();

    };

}

#endif