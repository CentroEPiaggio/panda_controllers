#include "../library/SelfMoveDistq.h"

/* Function name used to generate code */
#define GRADDISTQ_STRING "gradDistq_fun"
#define DOTGRADDISTQ_STRING "dotGradDistq_fun"

/* File name of generated code */
#define GENERATED_FILE "SelfMoveDistq_fun.cpp"

/* Define number of function generable */
#define NUMBER_FUNCTIONS 2

namespace regrob{
    
    SelfMoveDistq::SelfMoveDistq(){};

    SelfMoveDistq::SelfMoveDistq(const int numJoints, Eigen::VectorXd qmin_, Eigen::VectorXd qmax_, Eigen::VectorXd qbar_)
        :_numJoints_(numJoints){

        initVarsFuns();
        compute();
    }
    
    void SelfMoveDistq::init(const int numJoints, Eigen::VectorXd qmin_, Eigen::VectorXd qmax_, Eigen::VectorXd qbar_){

        _numJoints_ = numJoints;
        
        _qmin_ = qmin_;
        _qmax_ = qmax_;
        _qbar_ = qbar_;

        initVarsFuns();
        compute();        
    }
    
    void SelfMoveDistq::initVarsFuns(){
   
        _q_ = casadi::SX::sym("_q_", _numJoints_,1);
        _dq_ = casadi::SX::sym("_dq_", _numJoints_,1);

        q = Eigen::VectorXd::Zero(_numJoints_,1);
        dq = Eigen::VectorXd::Zero(_numJoints_,1);
        
        args.resize(5);
        
        for(int i=0;i<2;i++){
            args[i].resize(_numJoints_,1);
        }
        
        gradDistq_res.resize(1);
        dotGradDistq_res.resize(1);

        init_casadi_functions();
    }
    
    void SelfMoveDistq::init_casadi_functions(){
        
        gradDistq();
        dotGradDistq();
    }

    void SelfMoveDistq::gradDistq() {
        
        const int nj = _numJoints_;

        //casadi::SX H(1,1);
        casadi::SX dH(7,1);

        /* for (int i=0; i<_numJoints_;i++){
            H = H + pow((_q_(i)-_qbar_(i))/(_qmax_(i)-_qmin_(i)),2);
        }        
        dH = casadi::SX::jacobian(-0.5/_numJoints_*H, _q_);
        */

        for (int i=0; i<_numJoints_;i++){
            dH(i) = -(_q_(i)-_qbar_(i))/pow(_qmax_(i)-_qmin_(i),2)/nj;
        }
        
        casadi::Function my_fun(GRADDISTQ_STRING,{_q_},{densify(dH)});
        
        gradDistq_fun = my_fun;
    }

    void SelfMoveDistq::dotGradDistq() {
        
        const int nj = _numJoints_;

        //casadi::SX H(1,1);
        //casadi::SX dH(7,1);
        casadi::SX dotdH(7,1);

        /* for (int i=0; i<_numJoints_;i++){
            H = H + pow((_q_(i)-_qbar_(i))/(_qmax_(i)-_qmin_(i)),2);
        }        
        dH = casadi::SX::jacobian(-0.5/_numJoints_*H, _q_); 
        dotdH = jtimes(dH, _q_, _dq_); */

        for (int i=0; i<_numJoints_;i++){
            dotdH(i) = -(_dq_(i))/pow(_qmax_(i)-_qmin_(i),2)/nj;
        }
        
        casadi::Function my_fun(DOTGRADDISTQ_STRING,{_q_,_dq_},{densify(dotdH)});
        
        dotGradDistq_fun = my_fun;
    }

    void SelfMoveDistq::setArguments(const Eigen::VectorXd& q_){
        
        if(q_.size() == _numJoints_){
            q = q_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        compute();
    }

    void SelfMoveDistq::setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_){
        
        if(q_.size() == _numJoints_, dq_.size() == _numJoints_){
            q = q_;
            dq = dq_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        compute();
    }

    void SelfMoveDistq::compute(){
        for(int i=0;i<_numJoints_;i++){
            args[0](i,0) = q(i);
            args[1](i,0) = dq(i);
        }

        gradDistq_fun.call({args[0]},gradDistq_res);
        dotGradDistq_fun.call({args[0],args[1]},dotGradDistq_res);
    }
    
    Eigen::MatrixXd SelfMoveDistq::getGradDistq(){
        const int nrow = _numJoints_;
        const int ncol = 1;
        Eigen::MatrixXd dH_full(nrow,ncol);
        std::vector<casadi::SXElem> dH_elements = gradDistq_res[0].get_elements();
        
        std::transform(dH_elements.begin(), dH_elements.end(), dH_full.data(), mapFunction);
    
        return dH_full;
    }
    
    Eigen::MatrixXd SelfMoveDistq::getDotGradDistq(){
        const int nrow = _numJoints_;
        const int ncol = 1;
        Eigen::MatrixXd dotdH_full(nrow,ncol);
        std::vector<casadi::SXElem> dotdH_elements = dotGradDistq_res[0].get_elements();
        
        std::transform(dotdH_elements.begin(), dotdH_elements.end(), dotdH_full.data(), mapFunction);
    
        return dotdH_full;
    }

    void SelfMoveDistq::generate_code(std::string& savePath){
        
        // Options for c-code auto generation
        casadi::Dict opts = casadi::Dict();
        opts["cpp"] = true;
        opts["with_header"] = true;
        
        // generate functions in c code
        casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(GENERATED_FILE, opts);

        myCodeGen.add(gradDistq_fun);
        myCodeGen.add(dotGradDistq_fun);

        myCodeGen.generate(savePath);
    }

    std::vector<std::string> SelfMoveDistq::getFunctionsName() {
        std::vector<std::string> name_funs;
        name_funs.resize(NUMBER_FUNCTIONS);

        name_funs[0] = GRADDISTQ_STRING;
        name_funs[1] = DOTGRADDISTQ_STRING;

        return name_funs;
    }

    std::vector<casadi::Function> SelfMoveDistq::getCasadiFunctions() {
        std::vector<casadi::Function> casadi_funs;
        casadi_funs.resize(NUMBER_FUNCTIONS);

        casadi_funs[0] = gradDistq_fun;
        casadi_funs[1] = dotGradDistq_fun;

        return casadi_funs;
    }

}