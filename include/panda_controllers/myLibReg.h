#ifndef MYLIBREG_
#define MTLIBREG_

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>

namespace regrob{
    // Function to generate matrix template of transformation from Denavit-Hartenberg parameterization
    casadi::MX DHTemplate(const Eigen::MatrixXd& rowDHTable, const casadi::MX q, char jtsType);

    // Function for Forward Kinematic from Denavit-Hartenberg parameterization
    std::tuple<casadi::MXVector,casadi::MXVector> DHFwKin(const Eigen::MatrixXd& DHTable, const casadi::MX& q, const std::string& jtsType);

    // Function hat
    casadi::MX hat(const casadi::MX& v);

    // Function for Jacobian of link i-th from Denavit-Hartenberg parameterization
    std::tuple<casadi::MXVector,casadi::MXVector> DHJac(const casadi::MXVector& T0i_vec, const std::string& jtsType, const casadi::MX& baseOffset);

    // Function to create dq_selection matrix
    casadi::MX dq_select(const casadi::MX& dq);

    // Function to create E
    casadi::MXVector createE();

    // Function to obtain elements to construct C matrix with Christoffel symbols
    casadi::MXVector stdCmatrix(const casadi::MX& B, const casadi::MX& q, const casadi::MX& dq, casadi::MX& dq_sel);
        
    casadi::Function DHRegressor(const Eigen::MatrixXd& DH_table,const std::string& jType, const casadi::MX gravity);
}
#endif