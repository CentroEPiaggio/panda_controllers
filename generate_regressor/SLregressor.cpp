#include "SLregressor.h"

namespace regrob{
    
    /* Empty constructor */
    SLregressor::SLregressor(){}
    
    SLregressor::SLregressor(
        const int nj_, const Eigen::MatrixXd& DHTable_, const std::string jTypes_,
        FrameOffset& base_,FrameOffset& ee_, const bool dumped_): RegBasic(nj_), dumped(dumped_){
       
        dumped = dumped_;
        double mu =  dumped ? MU:MYZERO;
        std::cout<<"dumped: "<<dumped<<std::endl;
        std::cout<<"mu: "<<mu<<std::endl;

        //q = Eigen::VectorXd::Ones(numJoints)*M_PI_4;
        q = Eigen::VectorXd::Zero(numJoints);
        dq = Eigen::VectorXd::Zero(numJoints);
        dqr = Eigen::VectorXd::Zero(numJoints);
        ddqr = Eigen::VectorXd::Zero(numJoints);

        if(DHTable_.rows()==numJoints && jTypes_.size() == numJoints){
            DHTable = DHTable_;
            jointsTypes = jTypes_;
        }else{
            std::cout<<"in SLregress: invalid DHtable, or jointTypes dimensions \n";
        }
        
        args.resize(4);
        for(int i=0;i<4;i++){args[i].resize(numJoints,1);}
        
        regressor_res.resize(1);
        jacobian_res.resize(1);
        pinvJacobian_res.resize(1);
        dotPinvJacobian_res.resize(1);
        kinematic_res.resize(1);
        lab2L0 = base_;
        Ln2EE = ee_;
        
        matYr = SXregressor(DHTable,jTypes_,lab2L0,Ln2EE);
        regressor_fun = DHReg_fun(matYr);  
        
        jacobian_fun = DHJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE);
        pinvJacobian_fun = DHPinvJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE,mu);
        dotPinvJacobian_fun = DHDotPinvJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE,mu);

        kinematic_fun = DHKin_fun(DHTable,jointsTypes,lab2L0,Ln2EE);
        
        nonZeroCols.resize(matYr.size2());
        
        computeReg();
        computeJac();
        computeKin();
        
        //searchNonZero();
    }
    
    void SLregressor::init(int nj_, const Eigen::MatrixXd& DHTable_, const std::string jTypes_,
        FrameOffset& base_,FrameOffset& ee_,const bool dumped_){
        
        basic_init(nj_);
        double mu =  dumped ? MU:MYZERO;
        std::cout<<"dumped: "<<dumped<<std::endl;
        std::cout<<"mu: "<<mu<<std::endl;

        //q = Eigen::VectorXd::Ones(numJoints)*M_PI_4;
        q = Eigen::VectorXd::Zero(numJoints);
        dq = Eigen::VectorXd::Zero(numJoints);
        dqr = Eigen::VectorXd::Zero(numJoints);
        ddqr = Eigen::VectorXd::Zero(numJoints);

        if(DHTable_.rows()==numJoints && jTypes_.size() == numJoints){
            DHTable = DHTable_;
            jointsTypes = jTypes_;
        }else{
            std::cout<<"in SLregress: invalid DHtable, or jointTypes dimensions \n";
        }
        
        args.resize(4);
        for(int i=0;i<4;i++){args[i].resize(numJoints,1);}
        
        regressor_res.resize(1);
        jacobian_res.resize(1);
        pinvJacobian_res.resize(1);
        dotPinvJacobian_res.resize(1);
        kinematic_res.resize(1);

        lab2L0 = base_;
        Ln2EE = ee_;

        matYr = SXregressor(DHTable,jTypes_,lab2L0,Ln2EE);
        regressor_fun = DHReg_fun(matYr);  
        
        jacobian_fun = DHJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE);
        pinvJacobian_fun = DHPinvJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE,mu);
        dotPinvJacobian_fun = DHDotPinvJac_fun(DHTable,jointsTypes,lab2L0,Ln2EE,mu);

        kinematic_fun = DHKin_fun(DHTable,jointsTypes,lab2L0,Ln2EE);
        
        nonZeroCols.resize(matYr.size2());
        
        computeReg();
        computeJac();
        computeKin();
        
        //searchNonZero();
    }
    
    void SLregressor::computeReg(){
        for(int i=0;i<numJoints;i++){
            args[0](i,0) = q(i);
            args[1](i,0) = dq(i);
            args[2](i,0) = dqr(i);
            args[3](i,0) = ddqr(i);
        }
        //std::cout<<"in computeReg: check 0\n";
        regressor_fun.call(args,regressor_res);
        //std::cout<<"in computeReg: check 1\n";
    }
    void SLregressor::computeJac(){
        for(int i=0;i<numJoints;i++){
            args[0](i,0) = q(i);
            args[1](i,0) = dq(i);
        }
        //std::cout<<"in computeReg: check 0\n";
        jacobian_fun.call({args[0]},jacobian_res);
        pinvJacobian_fun.call({args[0]},pinvJacobian_res);
        dotPinvJacobian_fun.call({args[0],args[1]},dotPinvJacobian_res);
        //std::cout<<"in computeReg: check 1\n";
    }
    void SLregressor::computeKin(){
        for(int i=0;i<numJoints;i++){
            args[0](i,0) = q(i);
        }
        //std::cout<<"in computeReg: check 0\n";
        kinematic_fun.call({args[0]},kinematic_res);
        //std::cout<<"in computeReg: check 1\n";
    }
    /*void SLregressor::searchNonZero(){
        
        std::vector<casadi::SX> config(4);
        std::vector<casadi::SX> res;
        //std::vector<int> nonZeroCols;

        casadi::Slice allRow;
        casadi::SX qsym = casadi::SX::sym("q",numJoints,1);
        casadi::SX dqsym = casadi::SX::sym("dq",numJoints,1);
        casadi::SX dqrsym = casadi::SX::sym("dqr",numJoints,1);
        casadi::SX ddqrsym = casadi::SX::sym("ddqr",numJoints,1);
        
        int idx_arr = 0;

        config[0] = qsym;
        config[1] = dqsym;
        config[2] = dqrsym;
        config[3] = ddqrsym;
        
        regressor_fun.call(config,res);
        
        std::cout<<matYr.size()<<std::endl;
        
        std::vector<casadi::SXElem> matReg = res[0].get_elements();

        

        std::transform(matReg.begin(), matReg.end(), eigReg.data(), mapFunction);
        
        std::cout<<eigReg<<std::endl;
        
        for (int j = 0; j < eigReg.cols(); j++) {    
            if (eigReg.col(j).isZero(TOLLERANCE)) {   
                nonZeroCols[idx_arr]=j;
            }
            idx_arr++;
        }

        
    }*/
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
        //computeReg_gen();
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
    /*void SLregressor::setArgJac(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_){
        if(q_.size() == numJoints && dq_.size()==numJoints){
            q = q_;
            dq = dq_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computeJac();
    }*/
    Eigen::MatrixXd SLregressor::allColumns(){
        const int nrow = matYr.size1();
        const int ncol = matYr.size2();
        Eigen::MatrixXd Yfull(nrow,ncol);
        std::vector<casadi::SXElem> reg_elements = regressor_res[0].get_elements();
        std::transform(reg_elements.begin(), reg_elements.end(), Yfull.data(), mapFunction);
        
/*         for(int i=0;i<nrow;i++){
            for(int j=0;j<ncol;j++){ */
                /* if (abs((double)(regressor_res[0](i,j))) < TOLLERANCE){
                    regressor_res[0](i,j) = 0;
                } */
/*                 Yfull(i,j) = (double)(regressor_res[0](i,j));
            }
        } */
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
        
/*         for(int i=0;i<nrow;i++){
            for(int j=0;j<ncol;j++){ */
                /* if (abs((double)(jacobian_res[0](i,j))) < TOLLERANCE){
                    //std::cout<<"["<<i<<","<<j<<"]: "<<(double)(jacobian_res[0](i,j))<<std::endl;
                    jacobian_res[0](i,j) = 0;
                } */
/*                 Jacfull(i,j) = (double)(jacobian_res[0](i,j));
            }
        } */
        return Jacfull;
    }
    Eigen::MatrixXd SLregressor::pinvJacobian(){
        const int nrow = numJoints;
        const int ncol = 6;
        Eigen::MatrixXd pinvJacfull(nrow,ncol);
        std::vector<casadi::SXElem> pinvjac_elements = pinvJacobian_res[0].get_elements();
        std::transform(pinvjac_elements.begin(), pinvjac_elements.end(), pinvJacfull.data(), mapFunction);
        
/*         for(int i=0;i<nrow;i++){
            for(int j=0;j<ncol;j++){
                    if (abs((double)(jacobian_res[1](i,j))) < TOLLERANCE){
                        jacobian_res[1](i,j) = 0;
                    }
                Jacfull(i,j) = (double)(jacobian_res[1](i,j));
            }
        } */
        return pinvJacfull;
    }
    Eigen::MatrixXd SLregressor::dotPinvJacobian(){
        const int nrow = numJoints;
        const int ncol = 6;
        Eigen::MatrixXd dotPinvJacfull(nrow,ncol);
        std::vector<casadi::SXElem> dotpinvjac_elements = dotPinvJacobian_res[0].get_elements();
        
        std::transform(dotpinvjac_elements.begin(), dotpinvjac_elements.end(), dotPinvJacfull.data(), mapFunction);
        
//        for(int i=0;i<nrow;i++){
//            for(int j=0;j<ncol;j++){
                /* if (abs((double)(jacobian_res[2](i,j))) < TOLLERANCE){
                    jacobian_res[2](i,j) = 0;
                } */
//                Jacfull(i,j) = (double)(jacobian_res[2](i,j));
//            }
//        }
        return dotPinvJacfull;
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

        myCodeGen.generate(savePath);
    }

    double SLregressor::mapFunction(const casadi::SXElem& elem) {
        return static_cast<double>(casadi::SXElem(elem));
    }
}