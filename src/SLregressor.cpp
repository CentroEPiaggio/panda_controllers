#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>

#include "utils/SLregressor.h"

namespace regrob{
    #define TOLLERANCE 1e-15 // precision of regressor matrix
    SLregressor::SLregressor(){
    //SLregressor::SLregressor(const int nj_): numJoints(nj_){
        /*
        q = Eigen::VectorXd::Zero(numJoints);
        dq = Eigen::VectorXd::Zero(numJoints);
        dqr = Eigen::VectorXd::Zero(numJoints);
        ddqr = Eigen::VectorXd::Zero(numJoints);
        nonZeroCols.resize(numJoints*10);
         args.resize(4);
         for(int i=0;i<4;i++){args[i].resize(numJoints,1);}
         res.resize(1);
         */
    }
    SLregressor::SLregressor(const int nj_, const Eigen::MatrixXd& DHTable_, const std::string jTypes_,frame& base_):numJoints(nj_){
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
        lab2L0 = base_;

        regressor_fun = DHRegressor(DHTable,jointsTypes,lab2L0);
        
        compute();
        searchNonZero();
    }
    void SLregressor::init(int nj_, const Eigen::MatrixXd& DHTable_, const std::string jTypes_,frame& base_){
        numJoints = nj_;
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
        lab2L0 = base_;

        regressor_fun = DHRegressor(DHTable,jointsTypes,lab2L0);
        
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
        std::vector<casadi::SX> config(4);
        std::vector<casadi::SX> res;
        casadi::SX qsym = casadi::SX::sym("q",numJoints,1);
        casadi::SX dqsym = casadi::SX::sym("dq",numJoints,1);
        casadi::SX dqrsym = casadi::SX::sym("dqr",numJoints,1);
        casadi::SX ddqrsym = casadi::SX::sym("ddqr",numJoints,1);
        config[0] = qsym;
        config[1] = dqsym;
        config[2] = dqrsym;
        config[3] = ddqrsym;
        
        regressor_fun.call(config,res);

        for(int k=0;k<numJoints*10;k++){
            
            //res[0](allRow,k).simplify(qsym);
            //std::cout<<res[0](allRow,k)<<std::endl<<std::endl;
            nonZeroCols[k] = res[0](allRow,k).is_zero();
            //std::cout<<"\n["<<k<<"]: "<<"is zero? : "<<nonZeroCols[k]<<std::endl;
            //std::cout<<"["<<k<<"]: "<<res[0](allRow,k)<<std::endl;
            
        }
        //std::cout<<"\ncolonne nulle: \n"<<nonZeroCols<<std::endl<<std::endl;
    }
    void SLregressor::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_,const Eigen::VectorXd& dqr_,const Eigen::VectorXd& ddqr_){
            if(q_.size() == numJoints && dq_.size()==numJoints && dqr_.size()==numJoints && ddqr_.size()==numJoints){
                q = q_;
                dq = dq_;
                dqr = dqr_;
                ddqr = ddqr_;
            } else{
                std::cout<<"in setArguments: invalid dimensions of arguments\n";
            }
        compute();
    }
    Eigen::MatrixXd SLregressor::allColumns(){
        const int nrow = numJoints;
        const int ncol = numJoints*10;
        Eigen::MatrixXd Yfull(nrow,ncol);
        for(int i=0;i<nrow;i++){
            for(int j=0;j<ncol;j++){
                    if ((double)(res[0](i,j)) < TOLLERANCE){
                        res[0](i,j) = 0;
                    }
                Yfull(i,j) = (double)(res[0](i,j));
            }
        }
        return Yfull;
    }
    void SLregressor::generate_code(std::string& savePath){
    // Options for c-code auto generation
        casadi::Dict opts = casadi::Dict();
        opts["cpp"] = true;
        opts["with_header"] = true;
        
        // prefix for c code
        std::string prefix_code = savePath;
        // generate functions in c code
        casadi::CodeGenerator myCodeGen = casadi::CodeGenerator("gen_regr_fun.cpp", opts);
        myCodeGen.add(regressor_fun);

        myCodeGen.generate(prefix_code);
    }
}