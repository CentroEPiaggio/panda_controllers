#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>
#include <chrono>

#include "library/RobKinAdv.h"
#include "library/RobReg.h"
#include "library/SelfMoveDistq.h"

using namespace regrob;

using std::cout;
using std::endl;

int main(){

    /* Number of joints*/
    const int nj = 7;

    /* Denavit-Hartenberg Table of Franka
    Obtained from URDF file in $(find franka_gazebo)/test/launch/panda-gazebo.urdf */
    Eigen::Matrix<double,nj,4> DH_table;
    DH_table << 0,		-M_PI_2,	0.3330, 0,
                0,      M_PI_2,  	0,      0,
                0.0825, M_PI_2,  	0.3160, 0,
               -0.0825, -M_PI_2,	0,      0,
                0,      M_PI_2,  	0.384,  0,
                0.088,  M_PI_2,  	0,      0,
                0,      0,         	0.107,  0; 

    /* String of joints' type of robot, R for revolute and P for prismatic */
    std::string jType = "RRRRRRR"; 

    /* Define frame World to link 0 */
    FrameOffset base_to_L0({0,0,0},{0,0,0},{0,0,-9.81});

    /* Define frame end-effctor respect last joint
    Obtained in $(find franka_description)/robots/common/franka_hand.xacro */
    FrameOffset Ln_to_EE({-M_PI_4*0,0,0},{0,0,0.1034*0},{0,0,-9.81});
    //FrameOffset Ln_to_EE({0,0,0},{0,0,0.0},{0,0,-9.80});

    Eigen::VectorXd qmin(7), qmax(7), qbar(7);
    qmax << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    qmin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    qbar << 0, -0.785398163, 0, -2.35619449, 0, 1.57079632679, 0.785398163397;

    /* RobKin, RobReg, SelfMoveCost object */;
    RobKinAdv kinrobot;
    RobReg regrobot;
    SelfMoveDistq costdistq;

    kinrobot.init(nj,jType,DH_table,base_to_L0,Ln_to_EE, 0.001);
    regrobot.init(nj,jType,DH_table,base_to_L0,Ln_to_EE);
    costdistq.init(nj, qmin, qmax, qbar);

    /* Get Casadi Functions */
    std::vector<casadi::Function> kin_vec, reg_vec, cost_vec, all_vec;
    kin_vec = kinrobot.getCasadiFunctions();
    reg_vec = regrobot.getCasadiFunctions();    // only one function
    cost_vec = costdistq.getCasadiFunctions();

    /* Merge casadi function */
    int dim1, dim2, dim3;

    dim1 = kin_vec.size();
    dim2 = reg_vec.size();
    dim3 = cost_vec.size();

    for (int i=2; i<dim1; i++){     // exclude kinematic and jacobian
        all_vec.push_back(kin_vec[i]);
    }
    for (int i=0; i<dim2; i++){
        all_vec.push_back(reg_vec[i]);
    }
    for (int i=0; i<dim3; i++){
        all_vec.push_back(cost_vec[i]);
    }
    if(all_vec.size()!=dim1+dim2+dim3-2) cout<<"Merge Error"<<endl;

    /* Generate merge code */
    std::string relativePath = "../generatedFiles/";

    std::filesystem::path currentPath = std::filesystem::current_path();
    std::string absolutePath = currentPath / relativePath;
    std::cout << "Absolute path: " << absolutePath << std::endl;

    regrobot.generate_mergeCode(all_vec, absolutePath, "gen_regr_fun");

    std::filesystem::path headerPath = absolutePath + "gen_regr_fun.h";
    std::filesystem::path headerDestPath = "../../include/utils/gen_regr_fun.h";
    std::filesystem::copy_file(headerPath, headerDestPath, std::filesystem::copy_options::overwrite_existing);

    std::filesystem::path sourcePath = absolutePath + "gen_regr_fun.cpp";
    std::filesystem::path sourceDestPath = "../../src/gen_regr_fun.cpp";
    std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

    return 0;
}