#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>
#include <chrono>

#include "SLregressor.h"

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
    FrameOffset Ln_to_EE({-M_PI_4,0,0},{0,0,0.1034});
    
    /* Dumped flag is used to calculate pseudo-inverse of jacobian */
    const bool dumped = true;

    /* SLregressor object */;
    SLregressor regressore;
    regressore.init(nj,DH_table,jType,base_to_L0,Ln_to_EE,dumped);

    /* Generate code */
    std::string relativePath = "../ThunderGen/";

    std::filesystem::path currentPath = std::filesystem::current_path();
    std::string absolutePath = currentPath / relativePath;

    std::cout << "Percorso assoluto: " << absolutePath << std::endl;

    regressore.generate_code(absolutePath);
    
    /* ------------------- TEST ---------------------------*/
    
    /* Set a value to q, Dq, Dqr, DDqr, to evaluate regressor*/
    Eigen::VectorXd q = Eigen::VectorXd::Ones(nj)*M_PI_2;
    Eigen::VectorXd Dq = Eigen::VectorXd::Ones(nj)*0.05;
    Eigen::VectorXd Dqr = Eigen::VectorXd::Ones(nj)*0;
    Eigen::VectorXd DDqr = Eigen::VectorXd::Ones(nj)*0.1;

    /* Update regressor (using class SLregressor) */
    Eigen::Matrix<double,nj,nj*10> Yr;
    
    regressore.setArguments(q,Dq,Dqr,DDqr);
    Yr = regressore.allColumns();
    
    /* Display column regressor */
/*    for(int i=0;i<nj*10;i++){
         cout<<"\n["<<i<<"]"<<endl;
        cout<<Yr.col(i)<<endl;      
    } */

    return 0;
}