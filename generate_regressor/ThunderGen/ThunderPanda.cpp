#include "ThunderPanda.h"

namespace regrob{
    thunderPanda::thunderPanda(){};
    thunderPanda::thunderPanda(const int nj_):num_joints(nj_){
        q = Eigen::VectorXd::Zero(num_joints);
        dq = Eigen::VectorXd::Zero(num_joints);
        dqr = Eigen::VectorXd::Zero(num_joints);
        ddqr = Eigen::VectorXd::Zero(num_joints);

        reg_gen.resize(num_joints,10*num_joints);
        jac_gen.resize(6,7);
        pinvJac_gen.resize(7,6);
        dotPinvJac_gen.resize(7,6);
        kin_gen.resize(4,4);
        
    }
    void thunderPanda::init(const int nj_){
        num_joints = nj_;
        q = Eigen::VectorXd::Zero(num_joints);
        dq = Eigen::VectorXd::Zero(num_joints);
        dqr = Eigen::VectorXd::Zero(num_joints);
        ddqr = Eigen::VectorXd::Zero(num_joints);

        reg_gen.resize(num_joints,10*num_joints);
        jac_gen.resize(6,7);
        pinvJac_gen.resize(7,6);
        dotPinvJac_gen.resize(7,6);
        kin_gen.resize(4,4);
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
    }
    void thunderPanda::computeReg_gen(){
        
        long long int sz_arg;
        long long int sz_res;
        long long int sz_iw;
        long long int sz_w;

        int check_size = regr_fun_work(&sz_arg, &sz_res, &sz_iw, &sz_w);    
        
        long long p3[sz_iw];
        double p4[sz_w];

        const double* input_[] = {q.data(), dq.data(), dqr.data(), ddqr.data()};
        double* output_[] = {reg_gen.data()};

        int check = regr_fun(input_, output_, p3, p4, 0);

    }
    void thunderPanda::computeJac_gen(){
        
        long long int sz_arg;
        long long int sz_res;
        long long int sz_iw;
        long long int sz_w;

        int check_size = jac_fun_work(&sz_arg, &sz_res, &sz_iw, &sz_w);    
        
        long long p3[sz_iw];
        double p4[sz_w];

        const double* input_[] = {q.data()};
        double* output_[] = {jac_gen.data()};

        int check = jac_fun(input_, output_, p3, p4, 0);

    }
    void thunderPanda::computePinvJac_gen(){
        
        long long int sz_arg;
        long long int sz_res;
        long long int sz_iw;
        long long int sz_w;

        int check_size = pinvJac_fun_work(&sz_arg, &sz_res, &sz_iw, &sz_w);    
        
        long long p3[sz_iw];
        double p4[sz_w];

        const double* input_[] = {q.data()};
        double* output_[] = {pinvJac_gen.data()};

        int check = pinvJac_fun(input_, output_, p3, p4, 0);

    }
    void thunderPanda::computeDotPinvJac_gen(){
        
        long long int sz_arg;
        long long int sz_res;
        long long int sz_iw;
        long long int sz_w;

        int check_size = dotPinvJac_fun_work(&sz_arg, &sz_res, &sz_iw, &sz_w);    
        
        long long p3[sz_iw];
        double p4[sz_w];

        const double* input_[] = {q.data(),dq.data()};
        double* output_[] = {dotPinvJac_gen.data()};

        int check = dotPinvJac_fun(input_, output_, p3, p4, 0);

    }
    void thunderPanda::computeKin_gen(){
        
        long long int sz_arg;
        long long int sz_res;
        long long int sz_iw;
        long long int sz_w;

        int check_size = Kin_fun_work(&sz_arg, &sz_res, &sz_iw, &sz_w);    
        
        long long p3[sz_iw];
        double p4[sz_w];

        const double* input_[] = {q.data()};
        double* output_[] = {kin_gen.data()};

        int check = Kin_fun(input_, output_, p3, p4, 0);

    }
}