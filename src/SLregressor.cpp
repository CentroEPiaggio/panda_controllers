#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>

#include <panda_controllers/myLibReg.h>
#include <panda_controllers/SLregressor.h>

namespace regrob{
    SLregressor::SLregressor(int nj_, const Eigen::MatrixXd& DHTable_, const std::string jTypes_) : numJoints(nj_){
        q = Eigen::VectorXd::Zero(numJoints);
        dq = Eigen::VectorXd::Zero(numJoints);
        dqr = Eigen::VectorXd::Zero(numJoints);
        ddqr = Eigen::VectorXd::Zero(numJoints);
        nonZeroCols.resize(numJoints*10);
        
        if(DHTable_.rows()==numJoints && jTypes_.size() == numJoints){
            DHTable = DHTable_;
            jointsTypes = jTypes_;
        }else{
            std::cout<<"in SLregress: invalid DHtable, or jointTypes dimensions \n";
        }
        
         args.resize(4);
         for(int i=0;i<4;i++){
            args[i].resize(numJoints,1);
         }
         res.resize(1);
        
         regressor_fun = DHRegressor(DHTable,jointsTypes,casadi::DM {{0,0,-9.81}});
         
         compute();
         searchNonZero();
    }
    void SLregressor::compute(){
        for(int i=0;i<numJoints;i++){
            args[0](i,0) = q(i);
            args[1](i,0) = dq(i);
            args[2](i,0) = dqr(i);
            args[3](i,0) = ddqr(i);
        }
        regressor_fun.call(args,res);
    }
    void SLregressor::searchNonZero(){
        
        casadi::Slice allRow;
        std::vector<int> nonZeroCols(numJoints*10);
        std::vector<casadi::SX> config(4);
        std::vector<casadi::SX> res;
        config[0] = casadi::SX::sym("q",numJoints,1);
        config[1] = casadi::SX::sym("dq",numJoints,1);
        config[2] = casadi::SX::sym("dqr",numJoints,1);
        config[3] = casadi::SX::sym("ddqr",numJoints,1);
        
        regressor_fun.call(config,res);

        for(int k=0;k<numJoints*10;k++){
            nonZeroCols[k] = res[0](allRow,k).is_zero();
        }
        std::cout<<"\ncolonne nulle: \n"<<nonZeroCols<<std::endl<<std::endl;
    }
    void SLregressor::setArguments(const std::vector<Eigen::VectorXd>& arg_){
        if(arg_.size() == 4){
            if(arg_[0].size()==numJoints && arg_[1].size()==numJoints && arg_[2].size()==numJoints && arg_[3].size()==numJoints){
                q = arg_[0];
                dq = arg_[1];
                dqr = arg_[2];
                ddqr = arg_[3];
            } else{
                std::cout<<"in setArguments: invalid dimensions of arguments\n";
            }
        }else{
            std::cout<<"in setArguments: argument must be 4 \n";
        }
        compute();
    }
    Eigen::MatrixXd SLregressor::allColumns(){
        const int nrow = numJoints;
        const int ncol = numJoints*10;
        Eigen::MatrixXd Yfull(nrow,ncol);
        for(int i=0;i<nrow;i++){
            for(int j=0;j<ncol;j++){
                Yfull(i,j) = (double)(res[0](i,j));
            }
        }
        return Yfull;
    }
}