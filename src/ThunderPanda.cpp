#include "utils/ThunderPanda.h"
#include "utils/gen_regr_fun.h"

namespace regrob{
    
    thunderPanda::thunderPanda(){};

    thunderPanda::thunderPanda(const int nj_):num_joints(nj_){ 
        resizeVariables();
    }
   
    void thunderPanda::init(const int nj_){
        num_joints = nj_;
        resizeVariables();
    }

    void thunderPanda::resizeVariables(){
        
        q = Eigen::VectorXd::Zero(num_joints);
        dq = Eigen::VectorXd::Zero(num_joints);
        dqr = Eigen::VectorXd::Zero(num_joints);
        ddqr = Eigen::VectorXd::Zero(num_joints);

        reg_gen.resize(num_joints,10*num_joints);
        massReg_gen.resize(num_joints,10*num_joints);
        coriolisReg_gen.resize(num_joints,10*num_joints);
        gravityReg_gen.resize(num_joints,10*num_joints);
        jac_gen.resize(6,7);
        pinvJac_gen.resize(7,6);
        dotPinvJac_gen.resize(7,6);
        kin_gen.resize(4,4);
        gradDistq_gen.resize(7,1);
        dotGradDistq_gen.resize(7,1);
    }
    
    void thunderPanda::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_,const Eigen::VectorXd& dqr_,const Eigen::VectorXd& ddqr_){
        if(q_.size() == num_joints && dq_.size()== num_joints && dqr_.size()==num_joints && ddqr_.size()==num_joints){
            q = q_;
            dq = dq_;
            dqr = dqr_;
            ddqr = ddqr_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }

        computeReg_gen();
        computeMassReg_gen();
        computeCoriolisReg_gen();
        computeGravityReg_gen();
    }
    
    void thunderPanda::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_,const Eigen::VectorXd& dqr_){
        if(q_.size() == num_joints && dq_.size()== num_joints && dqr_.size()==num_joints){
            q = q_;
            dq = dq_;
            dqr = dqr_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }

        computeCoriolisReg_gen();
    }
    
    void thunderPanda::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_){
        if(q_.size() == num_joints && dq_.size()== num_joints){
            q = q_;
            dq = dq_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        //computeJac_gen();
        computeDotJac_gen();
        computePinvJac_gen();
        computeDotPinvJac_gen();
        //computeKin_gen();
        computeGradDistq_gen();
        computeDotGradDistq_gen();
    }
    
    void thunderPanda::setArguments(const Eigen::VectorXd& q_){
        if(q_.size() == num_joints){
            q = q_;
        } else{
            std::cout<<"in setArguments: invalid dimensions of arguments\n";
        }
        computeJac_gen();
        //computePinvJac_gen();
        computeKin_gen();
        computeGradDistq_gen();
    }
    
    void thunderPanda::computeReg_gen(){
        
        long long p3[regr_fun_SZ_IW];
        double p4[regr_fun_SZ_W];

        const double* input_[] = {q.data(), dq.data(), dqr.data(), ddqr.data()};
        double* output_[] = {reg_gen.data()};
        
        int check = regr_fun(input_, output_, p3, p4, 0);
        
    }
    
    void thunderPanda::computeMassReg_gen(){
        
        long long p3[massReg_fun_SZ_IW];
        double p4[massReg_fun_SZ_W];

        const double* input_[] = {q.data(), ddqr.data()};
        double* output_[] = {massReg_gen.data()};
        
        int check = massReg_fun(input_, output_, p3, p4, 0);
        
    }
    
    void thunderPanda::computeCoriolisReg_gen(){
        
        long long p3[coriolisReg_fun_SZ_IW];
        double p4[coriolisReg_fun_SZ_W];

        const double* input_[] = {q.data(), dq.data(), dqr.data()};
        double* output_[] = {coriolisReg_gen.data()};
        
        int check = coriolisReg_fun(input_, output_, p3, p4, 0);
        
    }

    void thunderPanda::computeGravityReg_gen(){
        
        long long p3[gravityReg_fun_SZ_IW];
        double p4[gravityReg_fun_SZ_W];

        const double* input_[] = {q.data()};
        double* output_[] = {gravityReg_gen.data()};
        
        int check = gravityReg_fun(input_, output_, p3, p4, 0);
        
    }

    void thunderPanda::computeJac_gen(){
        
        long long p3[jac_fun_SZ_IW];
        double p4[jac_fun_SZ_W];

        const double* input_[] = {q.data()};
        double* output_[] = {jac_gen.data()};

        int check = jac_fun(input_, output_, p3, p4, 0);

    }

    void thunderPanda::computeDotJac_gen(){
        
        long long p3[dotJac_fun_SZ_IW];
        double p4[dotJac_fun_SZ_W];

        const double* input_[] = {q.data(), dq.data()};
        double* output_[] = {dotJac_gen.data()};

        int check = dotJac_fun(input_, output_, p3, p4, 0);

    }

    void thunderPanda::computePinvJac_gen(){
        
        long long p3[pinvJac_fun_SZ_IW];
        double p4[pinvJac_fun_SZ_W];

        const double* input_[] = {q.data()};
        double* output_[] = {pinvJac_gen.data()};

        int check = pinvJac_fun(input_, output_, p3, p4, 0);

    }
    
    void thunderPanda::computeDotPinvJac_gen(){
        
        long long p3[dotPinvJac_fun_SZ_IW];
        double p4[dotPinvJac_fun_SZ_W];

        const double* input_[] = {q.data(),dq.data()};
        double* output_[] = {dotPinvJac_gen.data()};

        int check = dotPinvJac_fun(input_, output_, p3, p4, 0);

    }
    
    void thunderPanda::computeKin_gen(){
        
        long long p3[kin_fun_SZ_IW];
        double p4[kin_fun_SZ_W];

        const double* input_[] = {q.data()};
        double* output_[] = {kin_gen.data()};

        int check = kin_fun(input_, output_, p3, p4, 0);

    }

    void thunderPanda::computeGradDistq_gen(){
        
        long long p3[gradDistq_fun_SZ_IW];
        double p4[gradDistq_fun_SZ_W];

        const double* input_[] = {q.data()};
        double* output_[] = {gradDistq_gen.data()};

        int check = gradDistq_fun(input_, output_, p3, p4, 0);

    }

    void thunderPanda::computeDotGradDistq_gen(){
        
        long long p3[dotGradDistq_fun_SZ_IW];
        double p4[dotGradDistq_fun_SZ_W];

        const double* input_[] = {q.data(), dq.data()};
        double* output_[] = {dotGradDistq_gen.data()};

        int check = dotGradDistq_fun(input_, output_, p3, p4, 0);

    }

}