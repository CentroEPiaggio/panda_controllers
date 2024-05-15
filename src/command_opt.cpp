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
int l_idx;


Eigen::Matrix<double, NJ, 1> q_min_limit;
Eigen::Matrix<double, NJ, 1> q_max_limit;
Eigen::Matrix<double, NJ, 1> dq_limit;
Eigen::Matrix<double, NJ, 1> ddq_limit;
std::vector<double>  max_acc(7);
std::vector<double> min_acc(7);


using std::cout;
using std::cin;
using std::endl;

regrob::thunderPanda fastRegMat;

struct UserData {
    // std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    // std::vector<std::vector<double>> H;
    std::vector<double> H;
    int l;
};

UserData udata;

Eigen::VectorXd lbE;
Eigen::VectorXd ubE;

/*Funzioni necessarie per risoluzione problema di ottimo*/

void jointsCallback( const panda_controllers::udata::ConstPtr& msg );
double objective(const std::vector<double> &x, std::vector<double> &grad, void *data);
double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l);
// double constraintpos(const std::vector<double> &x, std::vector<double> &grad, void *data);
// double constraintvel(const std::vector<double> &x, std::vector<double> &grad, void *data);


int main(int argc, char **argv)
{
    // udata.q.resize(7);
    udata.dq.resize(7);
    udata.ddq.resize(7);
    udata.H.resize(700);
    lbE.resize(7);
    ubE.resize(7);
    // q_curr.setZero();
    l_idx = 1;

    fastRegMat.init(NJ);

    // q_max_limit << 2.0, 1.0, 2.0, -0.50, 2.50, 3.0, 2.0;
    // q_min_limit << -2.0, -1.0, -2.0, -2.50, -2.0, 0, -2.0;
    q_max_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
	q_min_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    dq_limit << 2.1, 2.1, 2.1, 2.1, 2.6, 2.6, 2.6;


    max_acc[0] = 15; max_acc[1] = 7.5; max_acc[2] = 10; max_acc[3] = 12.5; max_acc[4] = 15; max_acc[5] = 20; max_acc[6] = 20;
    min_acc[0] = -15; min_acc[1] = -7.5; min_acc[2] = -10; min_acc[3] = -12.5; min_acc[4] = -15; min_acc[5] = -20; min_acc[6] = -20;
    // max_acc[0] = 1.0; max_acc[1] = 1.0; max_acc[2] = 1.0; max_acc[3] = 1.0; max_acc[4] = 1.0; max_acc[5] = 1.0; max_acc[6] = 1.0;
    // min_acc[0] = -1.0; min_acc[1] = -1.0; min_acc[2] = -1.0; min_acc[3] = -1.0; min_acc[4] = -1.0; min_acc[5] = -1.0; min_acc[6] = -1.0;

    // lbE << -5, -2.5, -5, -5.5, -5, -10, -10;
    // ubE << 5, 2.5, 5, 5.5, 5, 10, 10;
    
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

    ros::Duration(1.0).sleep();

    while(ros::ok()){
        ros::spinOnce();

        // lbE << -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5;
        // ubE << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5;

        nlopt::opt opt(nlopt::algorithm::LN_COBYLA, NJ); // Algoritmo COYBLA -> GRADIENT FRRE OPTIMIZATION
        std::vector<double> x(7); // variabile soluzione di ottimo che viene inizializzata
        Eigen::VectorXd x_eig(7);
        
        udata.l = l_idx;
        if (l_idx <= 10){
            l_idx = l_idx + 2;
        }
        
        
        std::vector<double> lb(7), ub(7); // definisco lower and upper bound
        // std::vector<double> max_vel(7), max_pos(7), max_acc(7);
        // std::vector<double> min_vel(7), min_pos(7), min_acc(7);
        max_acc[0] = 15; max_acc[1] = 7.5; max_acc[2] = 10; max_acc[3] = 12.5; max_acc[4] = 15; max_acc[5] = 20; max_acc[6] = 20;
        min_acc[0] = -15; min_acc[1] = -7.5; min_acc[2] = -10; min_acc[3] = -12.5; min_acc[4] = -15; min_acc[5] = -20; min_acc[6] = -20;
        // max_acc[0] = 1.0; max_acc[1] = 1.0; max_acc[2] = 1.0; max_acc[3] = 1.0; max_acc[4] = 1.0; max_acc[5] = 1.0; max_acc[6] = 1.0;
        // min_acc[0] = -1.0; min_acc[1] = -1.0; min_acc[2] = -1.0; min_acc[3] = -1.0; min_acc[4] = -1.0; min_acc[5] = -1.0; min_acc[6] = -1.0;


        // cout << "position: "<<q_curr;
        // Sezione cating da Eigen a sdt perchè Eigen non compatibile con libreria di ottimo
        for(int i = 0; i < 7; ++i){
            // x[i] = ddq_curr[i];
            x[i] = q_curr(i);

            // lb[i] = ddq_curr(i)-0.01;
            // ub[i] = ddq_curr(i)+0.01;
            lb[i] = q_min_limit(i);
            ub[i] = q_max_limit(i);
            // ub[i] = (q_max_limit(i)-q_curr(i))/pow(0.001,2) - dq_curr(i)/0.001;
            // lb[i] = (q_min_limit(i)-q_curr(i))/pow(0.001,2) - dq_curr(i)/0.001;

            // udata.q[i] = q_curr(i);
            udata.dq[i] = dq_curr(i);
            udata.ddq[i] = ddq_curr(i);

            // cout << "---"<<*ub.data();

            // cout << "---"<<*ub.data();

            // ub[i] = std::min({max_acc[i], (dq_limit(i)-dq_curr(i))/0.001, (q_max_limit(i)-q_curr(i))/pow(0.001,2) - dq_curr(i)/0.001});
            // lb[i] = std::max({min_acc[i], (-dq_limit(i)-dq_curr(i))/0.001, (q_min_limit(i)-q_curr(i))/pow(0.001,2) - dq_curr(i)/0.001});
            // cout << "min: "<<min_vel[i] << "max: "<<max_vel[i];
            // cout << ub.data();
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
                x_eig(i) = x[i]; 
                command.position[i] = x[i];              
        }
        // mossa del vile(da cambire)
        // command.position[0] = minf;


        // cout<<"----"<< minf;
        

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

    // cout << "position: "<<q_curr;

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
    // Eigen::VectorXd q(7);
    Eigen::VectorXd dq(7);
    Eigen::VectorXd ddq(7);
    Eigen::VectorXd xe(7);
    int l;

    H_true.resize(10,70);

    l = udata->l;

    /*Recasting in Eigen*/
    for(int i=0; i<7; ++i){
        // q(i) = udata->q[i];
        dq(i) = udata->dq[i];
        ddq(i) = udata->ddq[i];
        xe(i) = x[i];
    }

    /*Prendiction command*/
    // dq = dq + xe*0.001;
    // q = q + dq*0.001;

    for(int i=0; i<70; ++i){
        H_true.block(0,i,PARAM,1) << udata->H[i*PARAM], udata->H[i*PARAM+1], udata->H[i*PARAM+2], udata->H[i*PARAM+3], udata->H[i*PARAM+4], udata->H[i*PARAM+5], udata->H[i*PARAM+6], udata->H[i*PARAM+7], udata->H[i*PARAM+8], udata->H[i*PARAM+9];
    }

    fastRegMat.setArguments(xe, dq, dq, ddq);
    Eigen::Matrix<double, NJ,PARAM*NJ> Y = fastRegMat.getReg_gen();
    Eigen::Matrix<double, NJ,PARAM> redY = Y.block(0,(NJ-1)*PARAM,NJ,PARAM);

    return -redStackCompute(redY, H_true, l);
    // Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_opt(H_true*H_true.transpose());
    
    // return -(solver_opt.singularValues()).minCoeff();
     
}

// double constraintpos1(const std::vector<double> &x, std::vector<double> &grad, void *data){
//         UserData *udata = reinterpret_cast<UserData*>(data);
//         x[0] - ;
// }

/*Provare facendo restituire direttamente il valore singolare*/
double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l){
    const int P = 10;
    const double epsilon = 0.1;
    double Vmax = 0;

    if (l <= (P-1)){
        if ((red_Y.transpose()-H.block(0,l*NJ,P,NJ)).norm()/(red_Y.transpose()).norm() >= epsilon){
            H.block(0,l*NJ,P,NJ) = red_Y.transpose();
            // l = l+1;
            // ROS_INFO_STREAM(H*H.transpose());
            Eigen::JacobiSVD<Eigen::Matrix<double, NJ*PARAM, NJ*PARAM>> solver_V(H*H.transpose());
            double V_max = (solver_V.singularValues()).minCoeff();
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