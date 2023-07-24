#include "SLregressor.h"

namespace regrob{
    
    SLregressor::SLregressor(){}
    
    SLregressor::SLregressor(
        const int nj_, const Eigen::MatrixXd& DHTable_, const std::string jTypes_,
        FrameOffset& base_,FrameOffset& ee_, const bool dumped_): RegBasic(nj_), dumped(dumped_){
       
        dumped = dumped_;
        double mu =  dumped ? MU:MYZERO;

        q = Eigen::VectorXd::Zero(numJoints);
        dq = Eigen::VectorXd::Zero(numJoints);
        dqr = Eigen::VectorXd::Zero(numJoints);
        ddqr = Eigen::VectorXd::Zero(numJoints);

        qbar = Eigen::VectorXd::Zero(numJoints);
        qmin = Eigen::VectorXd::Zero(numJoints);
        qmax = Eigen::VectorXd::Zero(numJoints);

        if(DHTable_.rows()==numJoints && jTypes_.size() == numJoints){
            DHTable = DHTable_;
            jointsTypes = jTypes_;
        }else{
            std::cout<<"in SLregress: invalid DHtable, or jointTypes dimensions \n";
        }
        
        args1.resize(4);
        args2.resize(4);
        for(int i=0;i<4;i++){
            args1[i].resize(numJoints,1);
            args2[i].resize(numJoints,1);
        }
        
        regressor_res.resize(1);
        jacobian_res.resize(1);
        pinvJacobian_res.resize(1);
        dotPinvJacobian_res.resize(1);
        kinematic_res.resize(1);

        dH_distqbar_res.resize(1);

        lab2L0 = base_;
        Ln2EE = ee_;
        
        matYr = SXregressor(DHTable,jTypes_,lab2L0,Ln2EE);
        
        regressor_fun = DHReg_fun(matYr);  
        jacobian_fun = DHJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE);
        pinvJacobian_fun = DHPinvJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE,mu);
        dotPinvJacobian_fun = DHDotPinvJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE,mu);
        kinematic_fun = DHKin_fun(DHTable,jointsTypes,lab2L0,Ln2EE);
        
        dH_distqbar_fun = dHDistFromq_fun();

        computeReg();
        computeJac();
        computeKin();
        computedHqbar();
    }
    
    void SLregressor::init(int nj_, const Eigen::MatrixXd& DHTable_, const std::string jTypes_,
        FrameOffset& base_,FrameOffset& ee_,const bool dumped_){
        
        basic_init(nj_);
        double mu =  dumped ? MU:MYZERO;

        q = Eigen::VectorXd::Zero(numJoints);
        dq = Eigen::VectorXd::Zero(numJoints);
        dqr = Eigen::VectorXd::Zero(numJoints);
        ddqr = Eigen::VectorXd::Zero(numJoints);

        qbar = Eigen::VectorXd::Zero(numJoints);
        qmin = Eigen::VectorXd::Zero(numJoints);
        qmax = Eigen::VectorXd::Zero(numJoints);

        if(DHTable_.rows()==numJoints && jTypes_.size() == numJoints){
            DHTable = DHTable_;
            jointsTypes = jTypes_;
        }else{
            std::cout<<"in SLregress: invalid DHtable, or jointTypes dimensions \n";
        }
        
        args1.resize(4);
        args2.resize(4);
        for(int i=0;i<4;i++){
            args1[i].resize(numJoints,1);
            args2[i].resize(numJoints,1);
        }
        
        regressor_res.resize(1);
        jacobian_res.resize(1);
        pinvJacobian_res.resize(1);
        dotPinvJacobian_res.resize(1);
        kinematic_res.resize(1);

        dH_distqbar_res.resize(1);

        lab2L0 = base_;
        Ln2EE = ee_;
        
        matYr = SXregressor(DHTable,jTypes_,lab2L0,Ln2EE);
        
        regressor_fun = DHReg_fun(matYr);  
        jacobian_fun = DHJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE);
        pinvJacobian_fun = DHPinvJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE,mu);
        dotPinvJacobian_fun = DHDotPinvJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE,mu);
        kinematic_fun = DHKin_fun(DHTable,jointsTypes,lab2L0,Ln2EE);
        
        dH_distqbar_fun = dHDistFromq_fun();

        computeReg();
        computeJac();
        computeKin();
        computedHqbar();
    }
    
    void SLregressor::computeReg(){
        for(int i=0;i<numJoints;i++){
            args1[0](i,0) = q(i);
            args1[1](i,0) = dq(i);
            args1[2](i,0) = dqr(i);
            args1[3](i,0) = ddqr(i);
        }
        regressor_fun.call(args1,regressor_res);
    }
    
    void SLregressor::computeJac(){
        for(int i=0;i<numJoints;i++){
            args1[0](i,0) = q(i);
            args1[1](i,0) = dq(i);
        }
        jacobian_fun.call({args1[0]},jacobian_res);
        pinvJacobian_fun.call({args1[0]},pinvJacobian_res);
        dotPinvJacobian_fun.call({args1[0],args1[1]},dotPinvJacobian_res);
    }
    
    void SLregressor::computeKin(){
        for(int i=0;i<numJoints;i++){
            args1[0](i,0) = q(i);
        }
        kinematic_fun.call({args1[0]},kinematic_res);
    }
    
    void SLregressor::computedHqbar(){
        for(int i=0;i<numJoints;i++){
            args2[0](i,0) = q(i);
            args2[1](i,0) = qbar(i);
            args2[2](i,0) = qmin(i);
            args2[3](i,0) = qmax(i);
        }
        dH_distqbar_fun.call(args2,dH_distqbar_res);
    }

    /*void SLregressor::reduceMinCols(){
        std::vector<int> colsIdx = nonZeroCols;
        casadi::SX reducedYr = matYr(casadi::Slice(), colsIdx);

        std::cout<<"Hello World " <<reducedYr<<std::endl;
        
        matYr = reducedYr;  // c'è da rinizializzare matYr
        
        std::cout<<matYr<<std::endl;
        
        regressor_fun = DHReg_fun(reducedYr);
        
        computeReg();
    }*/
    
    void SLregressor::setArguments(
        const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_,const Eigen::VectorXd& dqr_,const Eigen::VectorXd& ddqr_){
        if(q_.size() == numJoints && dq_.size()==numJoints && dqr_.size()==numJoints && ddqr_.size()==numJoints){
            q = q_;
            dq = dq_;
            dqr = dqr_;
            ddqr = ddqr_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computeReg();
    }
    
    void SLregressor::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_){
        if(q_.size() == numJoints && dq_.size()==numJoints){
            q = q_;
            dq = dq_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computeJac();
        computeKin();
    }
    
    void SLregressor::setArguments(const Eigen::VectorXd& q_){
        if(q_.size() == numJoints){
            q = q_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computeKin();
    }
    
    void SLregressor::setArgsdH(
        const Eigen::VectorXd& q_,const Eigen::VectorXd& q_bar_,const Eigen::VectorXd& q_min_,const Eigen::VectorXd& q_max_){
        if(q_.size() == numJoints && q_bar_.size()==numJoints && q_min_.size()==numJoints && q_max_.size()==numJoints){
            q = q_;
            qbar = q_bar_;
            qmin = q_min_;
            qmax = q_max_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computedHqbar();
    }

    Eigen::MatrixXd SLregressor::allColumns(){
        
        const int nrow = matYr.size1();
        const int ncol = matYr.size2();
        Eigen::MatrixXd Yfull(nrow,ncol);
        std::vector<casadi::SXElem> reg_elements = regressor_res[0].get_elements();
        std::transform(reg_elements.begin(), reg_elements.end(), Yfull.data(), mapFunction);
        
        return Yfull;
    }
    
    Eigen::Matrix4d SLregressor::kinematic(){

        Eigen::Matrix4d Kinfull;
        std::vector<casadi::SXElem> kin_elements = kinematic_res[0].get_elements();
        std::transform(kin_elements.begin(), kin_elements.end(), Kinfull.data(), mapFunction);
        
        return Kinfull;
    }
    
    Eigen::MatrixXd SLregressor::jacobian(){
        const int nrow = 6;
        const int ncol = numJoints;
        Eigen::MatrixXd Jacfull(nrow,ncol);
        std::vector<casadi::SXElem> jac_elements = jacobian_res[0].get_elements();
        std::transform(jac_elements.begin(), jac_elements.end(), Jacfull.data(), mapFunction);
        
        return Jacfull;
    }
    
    Eigen::MatrixXd SLregressor::pinvJacobian(){
        const int nrow = numJoints;
        const int ncol = 6;
        Eigen::MatrixXd pinvJacfull(nrow,ncol);
        std::vector<casadi::SXElem> pinvjac_elements = pinvJacobian_res[0].get_elements();
        std::transform(pinvjac_elements.begin(), pinvjac_elements.end(), pinvJacfull.data(), mapFunction);
    
        return pinvJacfull;
    }
    
    Eigen::MatrixXd SLregressor::dotPinvJacobian(){
        const int nrow = numJoints;
        const int ncol = 6;
        Eigen::MatrixXd dotPinvJacfull(nrow,ncol);
        std::vector<casadi::SXElem> dotpinvjac_elements = dotPinvJacobian_res[0].get_elements();
        
        std::transform(dotpinvjac_elements.begin(), dotpinvjac_elements.end(), dotPinvJacfull.data(), mapFunction);
    
        return dotPinvJacfull;
    }
    
    Eigen::MatrixXd SLregressor::dH_distqbar(){
        const int nrow = numJoints;
        const int ncol = 1;
        Eigen::MatrixXd dH_distqbar_full(nrow,ncol);
        std::vector<casadi::SXElem> dH_distqbar_elements = dH_distqbar_res[0].get_elements();
        
        std::transform(dH_distqbar_elements.begin(), dH_distqbar_elements.end(), dH_distqbar_elements.data(), mapFunction);
    
        return dH_distqbar_full;
    }
    
    //void SLregressor::setDumped(const double dumped_){dumped = dumped_;}
    
    void SLregressor::generate_code(std::string& savePath){
        
        // Options for c-code auto generation
        casadi::Dict opts = casadi::Dict();
        opts["cpp"] = true;
        opts["with_header"] = true;
        
        // generate functions in c code
        casadi::CodeGenerator myCodeGen = casadi::CodeGenerator("gen_regr_fun.cpp", opts);
        myCodeGen.add(regressor_fun);
        myCodeGen.add(jacobian_fun);
        myCodeGen.add(pinvJacobian_fun);
        myCodeGen.add(dotPinvJacobian_fun);
        myCodeGen.add(kinematic_fun);
        myCodeGen.add(dH_distqbar_fun);

        myCodeGen.generate(savePath);
    }

    double SLregressor::mapFunction(const casadi::SXElem& elem) {
        return static_cast<double>(casadi::SXElem(elem));
    }

}