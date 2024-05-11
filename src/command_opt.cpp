#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include "panda_controllers/point.h"
#include "panda_controllers/desTrajEE.h"
#include "panda_controllers/udata.h"

#include "utils/ThunderPanda.h"
// #include "utils/utils_cartesian.h"

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

int l;

using std::cout;
using std::cin;
using std::endl;

regrob::thunderPanda fastRegMat;

struct UserData {
    std::vector<double> q;
    std::vector<double> dq;
    // std::vector<std::vector<double>> H;
    std::vector<double> H;
};

UserData udata;

/*Funzioni necessarie per risoluzione problema di ottimo*/

void jointsCallback( const panda_controllers::udata::ConstPtr& msg );
double objective(const std::vector<double> &x, std::vector<double> &grad, void *data);
Eigen::MatrixXd redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l);

int main(int argc, char **argv)
{
    l = 7;
    udata.q.resize(7);
    udata.dq.resize(7);
    udata.H.resize(700);

    fastRegMat.init(NJ);
    
	ros::init(argc, argv, "command_opt");
	ros::NodeHandle node_handle;
    double frequency = 500;
	ros::Rate loop_rate(frequency); 
	
	/* Publisher */
	ros::Publisher pub_cmd_opt = node_handle.advertise<sensor_msgs::JointState>("command_joints_opt", 1);
	
	/* Subscriber */
	ros::Subscriber sub_config = node_handle.subscribe<panda_controllers::udata>("opt_data", 1, &jointsCallback);

    /* Commando di accelerazione*/
    sensor_msgs::JointState command;
    command.position.resize(NJ);
    command.velocity.resize(NJ);
    command.effort.resize(NJ);


    while(ros::ok()){
        ros::spinOnce();

        nlopt::opt opt(nlopt::algorithm::LN_COBYLA, NJ); // Algoritmo COYBLA -> GRADIENT FRRE OPTIMIZATION
        std::vector<double> x(7); // variabile soluzione di ottimo che viene inizializzata
        Eigen::VectorXd x_eig(7);

        
        std::vector<double> lb(7), ub(7); // definisco lower and upper bound

        // Sezione cating da Eigen a sdt perchè Eigen non compatibile con libreria di ottimo
        for(int i = 0; i < 7; ++i){
            x[i] = ddq_curr(i);
            lb[i] = ddq_curr(i)-0.1;
            ub[i] = ddq_curr(i)+0.1;
            udata.q[i] = q_curr(i);
            udata.dq[i] = dq_curr(i);
        }    

        // da verificare se corretta allocazione
        for(int i = 0; i < 700; ++i){
                udata.H[i] = H_vec(i);
        }

        opt.set_xtol_rel(1e-4);
        opt.set_min_objective(objective, &udata);  // definisco costo da massimizzare
        opt.set_lower_bounds(lb); // setto limite inferiore
        opt.set_upper_bounds(ub); // setto limite superiore 

        // Ottimizzazione
        double minf;
        nlopt::result result = opt.optimize(x, minf);
        for(int i = 0; i < 7; ++i){
                x_eig(i) = x[i]; 
                command.effort[i] = x_eig(i);              
        }

        // cout<<"valore di ottimo:"<< x_eig;
        

        // for(int i=0;i<NJ;i++){
        //     command.position[i] = x(i);
        //     command.velocity[i] = dot_qr(i);
        //     command.effort[i] = x[i];
        // }
        pub_cmd_opt.publish(command);
        loop_rate.sleep();
    }
    return 0;
}

void jointsCallback(const panda_controllers::udata::ConstPtr& msg){
   
    /*Devo inviare q future*/
	q_curr[0] = msg->q_cur[0];
	q_curr[1] = msg->q_cur[1];
	q_curr[2] = msg->q_cur[2];
	q_curr[3] = msg->q_cur[3];
	q_curr[4] = msg->q_cur[4];
	q_curr[5] = msg->q_cur[5];
	q_curr[6] = msg->q_cur[6];

    dq_curr[0] = msg->dot_q_curr[0];
	dq_curr[1] = msg->dot_q_curr[1];
	dq_curr[2] = msg->dot_q_curr[2];
	dq_curr[3] = msg->dot_q_curr[3];
	dq_curr[4] = msg->dot_q_curr[4];
	dq_curr[5] = msg->dot_q_curr[5];
	dq_curr[6] = msg->dot_q_curr[6];

    ddq_curr[0] = msg->ddot_q_curr[0];
	ddq_curr[1] = msg->ddot_q_curr[1];
	ddq_curr[2] = msg->ddot_q_curr[2];
	ddq_curr[3] = msg->ddot_q_curr[3];
	ddq_curr[4] = msg->ddot_q_curr[4];
	ddq_curr[5] = msg->ddot_q_curr[5];
	ddq_curr[6] = msg->ddot_q_curr[6];

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
    Eigen::VectorXd xe(7);

    H_true.resize(10,70);

    /*Recasting in Eigen*/
    for(int i=0; i<7; ++i){
        q(i) = udata->q[i];
        dq(i) = udata->dq[i];
        xe(i) = x[i];
    }

    for(int i=0; i<70; ++i){
        H_true.block(0,i,PARAM,1) << udata->H[i*PARAM], udata->H[i*PARAM+1], udata->H[i*PARAM+2], udata->H[i*PARAM+3], udata->H[i*PARAM+4], udata->H[i*PARAM+5], udata->H[i*PARAM+6], udata->H[i*PARAM+7], udata->H[i*PARAM+8], udata->H[i*PARAM+9];
    }

    fastRegMat.setArguments(q, dq, dq, xe);
    Eigen::Matrix<double, NJ,PARAM*NJ> Y = fastRegMat.getReg_gen();
    Eigen::Matrix<double, NJ,PARAM> redY = Y.block(0,(NJ-1)*PARAM,NJ,PARAM);

    redStackCompute(redY, H_true, l);
    Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_opt(H_true*H_true.transpose());
    
    return -(solver_opt.singularValues()).minCoeff();
     
}

Eigen::MatrixXd redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l){
    const int P = 10;
    const double epsilon = 0.1;

    if (l <= (P-1)){
        if ((red_Y.transpose()-H.block(0,l*NJ,P,NJ)).norm()/(red_Y.transpose()).norm() >= epsilon){
            H.block(0,l*NJ,P,NJ) = red_Y.transpose();
            // l = l+1;
            // ROS_INFO_STREAM(H*H.transpose());
        }
                            
    }else{
        if ((red_Y.transpose()-H.block(0,(P-1)*NJ,P,NJ)).norm()/(red_Y.transpose()).norm() >= epsilon){
            Eigen::MatrixXd Th = H;
            
            Eigen::JacobiSVD<Eigen::Matrix<double, NJ*PARAM, NJ*PARAM>> solver_V(H*H.transpose());
            double V = (solver_V.singularValues()).minCoeff();

            Eigen::VectorXd S(P);
            for (int i = 0; i < P; ++i) {
                H.block(0,i*NJ,P,NJ) = red_Y.transpose();
                Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_S(H*H.transpose());
                S(i) = (solver_S.singularValues()).minCoeff();
                H = Th;
            }
            double Vmax = S.maxCoeff();
            Eigen::Index m; //index max eigvalues 
            S.maxCoeff(&m);
            

            if(Vmax >= V){
                H.block(0,m*NJ,P,NJ) = red_Y.transpose();
            }
        }
    }
    return H;
}