#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include "panda_controllers/point.h"
// #include "panda_controllers/desTrajEE.h"
#include "panda_controllers/udata.h"
#include "panda_controllers/flag.h"

#include "utils/thunder_panda_2.h"
#include "utils/utils_cartesian.h"

#include "nlopt.hpp"

#ifndef     PARAM
# define    PARAM 10	// number of parameters for each link
#endif

#ifndef     NJ
# define    NJ 7	// number of joints
#endif



/*Variabili inizializzati*/
Eigen::VectorXd q_curr(7);
Eigen::VectorXd dq_curr(7);
Eigen::VectorXd ddq_curr(7);
Eigen::VectorXd H_vec(700);
int l_idx;


Eigen::Matrix<double, NJ, 1> q_c;
Eigen::Matrix<double, NJ, 1> q_min_limit;
Eigen::Matrix<double, NJ, 1> q_max_limit;
Eigen::Matrix<double, NJ, 1> dq_limit;
Eigen::Matrix<double, NJ, 1> ddq_limit;
std::vector<double>  max_acc(7);
std::vector<double> min_acc(7);


using std::cout;
using std::cin;
using std::endl;

thunder_ns::thunder_panda_2 fastRegMat;

struct UserData {
    // std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    // std::vector<std::vector<double>> H;
    std::vector<double> H;
    int l;
    double dt;
} udata;

/*Funzioni necessarie per risoluzione problema di ottimo*/

void jointsCallback( const panda_controllers::udata::ConstPtr& msg );
double objective(const std::vector<double> &x, std::vector<double> &grad, void *data);
double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l);


int main(int argc, char **argv)
{

    ros::init(argc, argv, "command_opt");
	ros::NodeHandle node_handle;
    double frequency = 500;
	ros::Rate loop_rate(frequency); 
	
    // udata.q.resize(7);
    udata.dq.resize(7);
    udata.ddq.resize(7);
    udata.H.resize(700);
    
    Eigen::Matrix<double, NJ, 1> qr;
    Eigen::Matrix<double, NJ, 1> dot_qr;
    Eigen::Matrix<double, NJ, 1> ddot_qr;
    // q_curr.setZero();
    l_idx = 11;

    // fastRegMat.init(NJ);

    // q_max_limit << 2.0, 1.0, 2.0, -0.50, 2.50, 3.0, 2.0;
    // q_min_limit << -2.0, -1.0, -2.0, -2.50, -2.0, 0, -2.0;
    q_c << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0;
    q_max_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
	q_min_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    dq_limit << 2.1, 2.1, 2.1, 2.1, 2.6, 2.6, 2.6;

    // lbE << -5, -2.5, -5, -5.5, -5, -10, -10;
    // ubE << 5, 2.5, 5, 5.5, 5, 10, 10;
    
	/* Publisher */
	ros::Publisher pub_cmd_opt = node_handle.advertise<sensor_msgs::JointState>("command_joints_opt", 1);
	ros::Publisher pub_flag_opt = node_handle.advertise<panda_controllers::flag>("/CT_mod_controller_OS/optFlag", 1);

	/* Subscriber */
	ros::Subscriber sub_config = node_handle.subscribe<panda_controllers::udata>("opt_data", 1, &jointsCallback);

    ros::Time t;
    double t_start;
    double dt = 0;
    // double wf = 20;

    t = ros::Time::now();
    t_start = t.toSec();

    /* Commando di accelerazione*/
    sensor_msgs::JointState command;
    panda_controllers::flag opt_flag_msg;
    command.position.resize(NJ);
    command.velocity.resize(NJ);
    command.effort.resize(NJ);
    std::vector<double> x(NJ); // variabile soluzione di ottimo che viene inizializzata
    // std::vector<double> x_old(7);

    // ros::Duration(0.1).sleep();

    while(ros::ok()){
        ros::spinOnce();
        qr.setZero();
        dot_qr.setZero();
        ddot_qr.setZero();
        opt_flag_msg.flag = true;

        t = ros::Time::now();
        if (dt == 0)
        {
            dt = t.toSec() - t_start;
            cout<<"tempo di inizio:"<<dt;
        }else
        {
            dt = t.toSec() - t_start;
        }
        udata.dt = dt;

        // lbE << -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5;
        // ubE << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5;

        nlopt::opt opt(nlopt::algorithm::LN_COBYLA, NJ); // Algoritmo COYBLA -> GRADIENT FRRE OPTIMIZATION
        
        
        udata.l = 11;
        
        
         // definisco lower and upper bound
        std::vector<double> lb(NJ), ub(NJ);
        // max_acc[0] = 15; max_acc[1] = 7.5; max_acc[2] = 10; max_acc[3] = 12.5; max_acc[4] = 15; max_acc[5] = 20; max_acc[6] = 20;
        // min_acc[0] = -15; min_acc[1] = -7.5; min_acc[2] = -10; min_acc[3] = -12.5; min_acc[4] = -15; min_acc[5] = -20; min_acc[6] = -20;
      

        /*Bound for omega*/
        lb[0] = -M_PI_2; lb[1] = -M_PI_2; lb[2] = -M_PI; lb[3] = -M_PI; lb[4] = -M_PI_2; lb[5] = -M_PI_2; lb[6] = -M_PI;
        ub[0] = M_PI_2; ub[1] = M_PI_2; ub[2] = M_PI; ub[3] = M_PI; ub[4] = M_PI_2; ub[5] = M_PI_2; ub[6] = M_PI;

        /*Bound for A*/
        //  for (int i = 0; i < NJ; ++i) {
        //     lb[NJ + i] = q_min_limit(i) / 2;
        //     ub[NJ + i] = q_max_limit(i) / 2;
        // }

        // cout << "position: "<<q_curr;
        // Sezione cating da Eigen a sdt perchè Eigen non compatibile con libreria di ottimo
        for(int i=0; i < 7; ++i){
            x[i] = 0;
            // x[i+NJ] = (q_min_limit(i)+q_max_limit(i))/4;
            // for(int r=0; r<2; ++r){
            //     x[4*i+2*r] = 0;
            //     x[4*i+2*r+1] = 0;
            //     lb[4*i+2*r] = std::max(0.5*wf*r*q_min_limit(i), -dq_limit(i));
            //     lb[4*i+2*r+1] = lb[4*i+2*r];
            //     ub[4*i+2*r] = std::min(0.5*wf*r*q_max_limit(i), dq_limit(i));
            //     ub[4*i+2*r+1] = ub[4*i+2*r];
            // }
           
        }    

        /*Calcolo dei bound*/


        // da verificare se corretta allocazione
        for(int i = 0; i < 700; ++i){
                udata.H[i] = H_vec(i);
        }

        opt.set_xtol_rel(1e-4);
        opt.set_lower_bounds(lb); // setto limite inferiore
        opt.set_upper_bounds(ub); // setto limite superiore 
        opt.set_min_objective(objective, &udata);  // definisco costo da massimizzare
    

        // Ottimizzazione
        double minf;
        nlopt::result result = opt.optimize(x, minf);
        for(int i = 0; i < 7; ++i){
                qr(i) = q_c(i) + 0.30*sin(x[i]*dt);     
                dot_qr(i) = x[i]*0.30*cos(x[i]*dt);
                ddot_qr(i) = -x[i]*x[i]*0.30*sin(x[i]*dt);      
        }
        


        // qr << 0.0+0.30*sin(1.5*(x[0])*dt), 0.0+0.30*sin(2*(x[1])*dt), 0.0+0.30*sin(2*(x[2])*dt), -1.5+0.30*sin(2*(x[3])*dt), 0.0+0.30*sin(2*(x[4])*dt), 1.5+0.30*sin(2*(x[5])*dt), 0.0+0.30*sin(2*(x[6])*dt);
        // dot_qr << 1.5*(x[0])*0.30*cos(1.5*(x[0])*dt), 2*(x[1])*0.30*cos(2*(x[1])*dt), 2*(x[2])*0.30*cos(2*(x[2])*dt), 2*(x[3])*0.30*cos(2*(x[3])*dt), 2*(x[4])*0.30*cos(2*(x[4])*dt), 2*(x[5])*0.30*cos(2*(x[5])*dt), 2*(x[6])*0.30*cos(2*(x[6])*dt);
        // ddot_qr << -pow(1.5*(x[0]),2)*0.30*sin(1.5*(x[0])*dt), -pow(2*(x[1]),2)*0.30*sin(2*(x[1])*dt), +pow(2*(x[2]),2)*0.30*sin(2*(x[2])*dt), -pow(2*(x[3]),2)*0.30*sin(2*(x[3])*dt), -pow(2*(x[4]),2)*0.30*sin(2*(x[4])*dt), -pow(2*(x[5]),2)*0.30*sin(2*(x[5])*dt), -pow(2*(x[6]),2)*0.30*sin(2*(x[6])*dt);
    
        // for(int i=0; i<NJ; ++i){
        //     for(int r=0; r<2; ++r){
        //         qr(i) = qr(i) + x[4*i+2*r]/(wf*r)*sin(wf*r*dt) - x[4*i+2*r+1]/(wf*r)*cos(wf*r*dt);
        //         dot_qr(i) = dot_qr(i) + x[4*i+2*r]*cos(wf*r*dt) + x[4*i+2*r+1]*sin(wf*r*dt);
        //         ddot_qr(i) = ddot_qr(i) - wf*r*x[4*i+2*r]*sin(wf*r*dt) + wf*r*x[4*i+2*r+1]*cos(wf*r*dt);
        //     }
        // }


        for(int i=0;i<NJ;i++){
            command.position[i] = qr(i);
            command.velocity[i] = dot_qr(i);
            command.effort[i] = ddot_qr(i);
        }

        pub_flag_opt.publish(opt_flag_msg);
        pub_cmd_opt.publish(command);
        loop_rate.sleep();
    }
    return 0;
}

void jointsCallback(const panda_controllers::udata::ConstPtr& msg){
   
    /*Devo inviare q future*/
	// q_curr[0] = msg->q_cur[0];
	// q_curr[1] = msg->q_cur[1];
	// q_curr[2] = msg->q_cur[2];
	// q_curr[3] = msg->q_cur[3];
	// q_curr[4] = msg->q_cur[4];
	// q_curr[5] = msg->q_cur[5];
	// q_curr[6] = msg->q_cur[6];

    // // cout << "position: "<<q_curr;

    // dq_curr[0] = msg->dot_q_curr[0];
	// dq_curr[1] = msg->dot_q_curr[1];
	// dq_curr[2] = msg->dot_q_curr[2];
	// dq_curr[3] = msg->dot_q_curr[3];
	// dq_curr[4] = msg->dot_q_curr[4];
	// dq_curr[5] = msg->dot_q_curr[5];
	// dq_curr[6] = msg->dot_q_curr[6];

    // ddq_curr[0] = msg->ddot_q_curr[0];
	// ddq_curr[1] = msg->ddot_q_curr[1];
	// ddq_curr[2] = msg->ddot_q_curr[2];
	// ddq_curr[3] = msg->ddot_q_curr[3];
	// ddq_curr[4] = msg->ddot_q_curr[4];
	// ddq_curr[5] = msg->ddot_q_curr[5];
	// ddq_curr[6] = msg->ddot_q_curr[6];

    for(int i = 0; i<70; ++i){
        H_vec.segment(i*PARAM, PARAM) << msg->H_stack[i*PARAM], msg->H_stack[i*PARAM+1], msg->H_stack[i*PARAM+2], msg->H_stack[i*PARAM+3], msg->H_stack[i*PARAM+4], msg->H_stack[i*PARAM+5], msg->H_stack[i*PARAM+6], msg->H_stack[i*PARAM+7], msg->H_stack[i*PARAM+8], msg->H_stack[i*PARAM+9];
    }

    // cout<<"valore di ottimo:"<< H_vec;

	// if (!init_start){
	// 	q_start = q_curr;
	// 	init_start = true;
	// }
}

double objective(const std::vector<double> &x, std::vector<double> &grad, void *data){

    UserData *udata = reinterpret_cast<UserData*>(data);
    Eigen::MatrixXd H_true;
    Eigen::VectorXd q(7);
    Eigen::VectorXd dq(7);
    Eigen::VectorXd ddq(7);
    // double wf = 20;
    // Eigen::VectorXd x[7];
    int l;
    double dt;

    H_true.resize(10,70);

    l = udata->l;
    dt = udata->dt;


    if (!grad.empty()) {
        for (int i = 0; i < NJ; i++) {
            grad[i] = 0.0;
        }
    }


    for(int i = 0; i < NJ; ++i){
            q(i) = q_c(i) + 0.30*sin(x[i]*dt);     
            dq(i) = x[i]*0.30*cos(x[i]*dt);
            ddq(i) = -x[i]*x[i]*0.30*sin(x[i]*dt);      
    }

    // q << 0.0+0.30*sin(1.5*(x[0])*dt), 0.0+0.30*sin(2*(x[1])*dt), 0.0+0.30*sin(2*(x[2])*dt), -1.5+0.30*sin(2*(x[3])*dt), 0.0+0.60*sin(2*(x[4])*dt), 1.5+0.60*sin(2*(x[5])*dt), 0.0+0.60*sin(2*(x[6])*dt);
    // dq << 1.5*(x[0])*0.30*cos(1.5*(x[0])*dt), 2*(x[1])*0.30*cos(2*(x[1])*dt), 2*(x[2])*0.30*cos(2*(x[2])*dt), 2*(x[3])*0.30*cos(2*(x[3])*dt), 2*(x[4])*0.60*cos(2*(x[4])*dt), 2*(x[5])*0.60*cos(2*(x[5])*dt), 2*(x[6])*0.60*cos(2*(x[6])*dt);
    // ddq << -pow(1.5*(x[0]),2)*0.30*sin(1.5*(x[0])*dt), -pow(2*(x[1]),2)*0.30*sin(2*(x[1])*dt), +pow(2*(x[2]),2)*0.30*sin(2*(x[2])*dt), -pow(2*(x[3]),2)*0.30*sin(2*(x[3])*dt), -pow(2*(x[4]),2)*0.60*sin(2*(x[4])*dt), -pow(2*(x[5]),2)*0.60*sin(2*(x[5])*dt), -pow(2*(x[6]),2)*0.60*sin(2*(x[6])*dt);
    
    // q.setZero();
    // dq.setZero();
    // ddq.setZero();
    /*Serie di Fourier ordine r*/
    // for(int i=0; i<NJ; ++i){
    //     for(int r=0; r<2; ++r){
    //         q(i) = q(i) + x[4*i+2*r]/(wf*r)*sin(wf*r*dt) - x[4*i+2*r+1]/(wf*r)*cos(wf*r*dt);
    //         dq(i) = dq(i) + x[4*i+2*r]*cos(wf*r*dt) + x[4*i+2*r+1]*sin(wf*r*dt);
    //         ddq(i) = ddq(i) - wf*r*x[4*i+2*r]*sin(wf*r*dt) + wf*r*x[4*i+2*r+1]*cos(wf*r*dt);
    //     }
    // }
    

    for(int i=0; i<70; ++i){
        H_true.block(0,i,PARAM,1) << udata->H[i*PARAM], udata->H[i*PARAM+1], udata->H[i*PARAM+2], udata->H[i*PARAM+3], udata->H[i*PARAM+4], udata->H[i*PARAM+5], udata->H[i*PARAM+6], udata->H[i*PARAM+7], udata->H[i*PARAM+8], udata->H[i*PARAM+9];
    }

    fastRegMat.setArguments(q, dq, dq, ddq);
    Eigen::Matrix<double, NJ,PARAM*NJ> Y = fastRegMat.getReg();
    Eigen::Matrix<double, NJ,PARAM> redY = Y.block(0,(NJ-1)*PARAM,NJ,PARAM);

    return -redStackCompute(redY, H_true, l);
    // Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_opt(H_true*H_true.transpose());
    
    // return -(solver_opt.singularValues()).minCoeff();
     
}


double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l){
    const int P = 10;
    const double epsilon = 0.1;
    double Vmax = 0;

    if (l <= (P-1)){
        if ((red_Y.transpose()-H.block(0,l*NJ,P,NJ)).norm()/(red_Y.transpose()).norm() >= epsilon){
            H.block(0,l*NJ,P,NJ) = red_Y.transpose();
            // l = l+1;
            // ROS_INFO_STREAM(H*H.transpose());
            Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_V(H*H.transpose());
            double V_max = (solver_V.singularValues()).minCoeff();
        }
                            
    }else{
        if ((red_Y.transpose()-H.block(0,(P-1)*NJ,P,NJ)).norm()/(red_Y.transpose()).norm() >= epsilon){
            Eigen::MatrixXd Th = H;
            
            Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_V(H*H.transpose());
            double V = (solver_V.singularValues()).minCoeff();

            Eigen::VectorXd S(P);
            for (int i = 0; i < P; ++i) {
                H.block(0,i*NJ,P,NJ) = red_Y.transpose();
                Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_S(H*H.transpose());
                S(i) = (solver_S.singularValues()).minCoeff();
                H = Th;
            }
            Vmax = S.maxCoeff();
            Eigen::Index m; //index max eigvalues 
            S.maxCoeff(&m);
            

            if(Vmax >= V){
                H.block(0,m*NJ,P,NJ) = red_Y.transpose();
            }else{
                Vmax = V;
            }
        }
    }
    return Vmax;
}