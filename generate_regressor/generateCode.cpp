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

    const int nj = 7;
    Eigen::Matrix<double,nj,4> DH_table;

    // DH Table franka
/*     DH_table << 0,        -M_PI_2,  0.3330,     0,
                0,         M_PI_2,  0,          -M_PI_4,
                0.0825,    M_PI_2,  0.3160,     0,
               -0.0825,   -M_PI_2,  0,          -3*M_PI_4,
                0,         M_PI_2,  0.384,      0,
                0.088,     M_PI_2,  0,          M_PI_2,
                0,         0,         0.107,    M_PI_4;  */
    DH_table << 0,		-M_PI_2,	0.3330, 0,
                0,      M_PI_2,  	0,      0,
                0.0825, M_PI_2,  	0.3160, 0,
               -0.0825, -M_PI_2,	0,      0,
                0,      M_PI_2,  	0.384,  0,
                0.088,  M_PI_2,  	0,      0,
                0,      0,         	0.107,  0; 
    std::string jType = "RRRRRRR"; // Stringa

    Eigen::Matrix<double,nj,nj*10> Yr;
    

/*     const int nj = 2;
    Eigen::Matrix<double,nj,4> DH_table;
    // DH table RR planare
    DH_table << 1.2, M_PI/2,  0,  0,
                1.2, M_PI/2,  0,  0;
    std::string jType = "RR"; // Stringa */

    /* INIZIALIZZAZIONE FRAME L0 RISPETTO A FRAME WORLD*/
    FrameOffset base_to_L0({0,0,0},{0,0,0},{0,0,-9.81});
    FrameOffset Ln_to_EE({-M_PI_4,0,0},{0,0,0.1034});
    const bool dumped = true;

    //SLregressor regressore(nj,DH_table,jType,base_to_L0,Ln_to_EE,dumped);
    SLregressor regressore;
    
    regressore.init(nj,DH_table,jType,base_to_L0,Ln_to_EE,dumped);

    std::string savePath = "/home/paolo/Documents/robotica/progettino_robotica/test_3_code_gen/ThunderGen/";
    regressore.generate_code(savePath);
    
    cout<<"\n\nOUTPUT TESTS AFTER A SetArgument()\n\n";
    Eigen::VectorXd q = Eigen::VectorXd::Ones(nj)*M_PI_2;
    Eigen::VectorXd Dq = Eigen::VectorXd::Ones(nj)*0.05;
    Eigen::VectorXd Dqr = Eigen::VectorXd::Ones(nj)*0;
    Eigen::VectorXd DDqr = Eigen::VectorXd::Ones(nj)*0.1;

    regressore.setArguments(q,Dq,Dqr,DDqr);
    Yr = regressore.allColumns();
    
    // display column regressor
    for(int i=0;i<nj*10;i++){
        cout<<"\n["<<i<<"]"<<endl;
        cout<<Yr.col(i)<<endl;      
    }
    // -------------------------


    return 0;
}