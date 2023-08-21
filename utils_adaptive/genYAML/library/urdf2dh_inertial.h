#ifndef URDF2DH_INERTIAL
#define URDF2DH_INERTIAL

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace regrob{

    struct LinkProp {

        double mass;
        std::vector<double> parI = std::vector<double>(6); // Inertia in the order xx, xy, xz, yy, yz, zz
        std::vector<double> xyz = std::vector<double>(3); // Origin xyz as std::vector
        std::vector<double> rpy = std::vector<double>(3);
        std::string name;
    };


    struct urdf2dh_T{
        std::vector<double> xyz;
        std::vector<double> rpy;

    };

    void trasformBodyInertial(std::vector<double> d_i, std::vector<double> rpy_i, const LinkProp body_urdf, LinkProp &body);
    Eigen::Matrix3d rpyRot(const std::vector<double> rpy);
    Eigen::Matrix3d createI(const std::vector<double> parI);
    Eigen::Matrix3d hat(const Eigen::Vector3d v);
}



#endif