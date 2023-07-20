#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <chrono>

#include "ThunderPanda.h"

using namespace regrob;
using std::cout;
using std::endl;

int main(){

    const int nj = 7;

    Eigen::Matrix<double,nj,nj*10> Yr;
    
    thunderPanda regressore;
    regressore.init(nj);
    
    cout<<"\n\nOUTPUT TESTS AFTER A SetArgument()\n\n";
    Eigen::VectorXd q = Eigen::VectorXd::Ones(nj)*M_PI_2;
    Eigen::VectorXd Dq = Eigen::VectorXd::Ones(nj)*0.05;
    Eigen::VectorXd Dqr = Eigen::VectorXd::Ones(nj)*0;
    Eigen::VectorXd DDqr = Eigen::VectorXd::Ones(nj)*0.1;

    regressore.setArguments(q,Dq,Dqr,DDqr);
    Yr = regressore.getReg_gen();
    
    // display column regressor
    for(int i=0;i<nj*10;i++){
        cout<<"\n["<<i<<"]"<<endl;
        cout<<Yr.col(i)<<endl;      
    }
    // -------------------------


    return 0;
}