#ifndef MYLIBREG_
#define MTLIBREG_

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>

typedef std::vector<double> vec3d;

namespace regrob{
        /* ========== CLASSE FRAME DEFINIZIONE ========== */
    class frame {
        private:
            vec3d ypr;
            vec3d position;
            vec3d gravity;
        public:
            frame();
            frame(const vec3d& ,const vec3d&,const vec3d&);
            ~frame(){std::cout<<"frame eliminato\n";};
            casadi::MX rotation();
            casadi::MX transform();
            //casadi::MX get_position();
            //casadi::MX get_gravity();
            void set_position(const vec3d&);
            void set_ypr(const vec3d&);
            void set_gravity(const vec3d&);
            casadi::MX hat(const vec3d&);
            casadi::MX get_position();
            vec3d get_ypr();
            casadi::MX get_gravity();
    };

    // Function to generate matrix template of transformation from Denavit-Hartenberg parameterization
    casadi::MX DHTemplate(const Eigen::MatrixXd& rowDHTable, const casadi::MX q, char jtsType);

    // Function for Forward Kinematic from Denavit-Hartenberg parameterization
    std::tuple<casadi::MXVector,casadi::MXVector> DHFwKin(const Eigen::MatrixXd& DHTable, const casadi::MX& q, const std::string& jtsType);

    // Function hat
    casadi::MX hat(const casadi::MX& v);

    // Function for Jacobian of link i-th from Denavit-Hartenberg parameterization
    std::tuple<casadi::MXVector,casadi::MXVector> DHJac(const casadi::MXVector& T0i_vec, const std::string& jtsType,frame&);

    // Function to create dq_selection matrix
    casadi::MX dq_select(const casadi::MX& dq);

    // Function to create E
    casadi::MXVector createE();

    // Function to obtain elements to construct C matrix with Christoffel symbols
    casadi::MXVector stdCmatrix(const casadi::MX& B, const casadi::MX& q, const casadi::MX& dq, casadi::MX& dq_sel);
        
    casadi::Function DHRegressor(const Eigen::MatrixXd& DH_table,const std::string& jType,frame&);
    
}
#endif