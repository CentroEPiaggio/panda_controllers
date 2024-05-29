#include <iostream>
#include <eigen3/Eigen/Dense>
#include <utils/min_jerk.h>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>
#include "panda_controllers/desTrajEE.h"
#include "panda_controllers/flag.h"
#include "panda_controllers/udata.h"

#include "ros/ros.h"
// #include "panda_controllers/CommandParams.h"
// #include "panda_controllers/Commands.h"

#include <sstream>

/*Libreria per ottimo*/
#include "utils/thunder_panda_2.h"
#include "utils/utils_cartesian.h"
#include "nlopt.hpp"

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
// #include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "franka_msgs/FrankaState.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// // #define ROBOT_NAME "/robot/arm"	// real robot
// #define ROBOT_NAME ""	// simulation

#ifndef     PARAM
# define    PARAM 10	// number of parameters for each link
#endif

#ifndef     NJ
# define    NJ 7	// number of parameters for each link
#endif

using namespace std;

#define alpha 0.1
bool ready = false;
bool init_q = false;
bool init_p = false;
std:: string robot_name;
// define q as 7x1 matrix
Eigen::Matrix<double, 7, 1> q;
Eigen::Vector3d p;
Eigen::Matrix<double, 4, 1> orient;
Eigen::Vector3d p_start;	// used to save starting point of tragectory (go to start)
Eigen::Vector3d p_end;	// used to save starting point of tragectory (go to start)
min_jerk_class min_jerk;
// franka_throw_class throw_node = franka_throw_class();
// lissagious parameters
double ampX, ampY, ampZ, freqX, freqY, freqZ, phiX, phiZ, offX, offY, offZ, liss_T;
/*Opt variable*/
Eigen::VectorXd H_vec(700);
thunder_ns::thunder_panda_2 fastRegMat;
Eigen::Matrix<double, NJ, 1> q_c;

ros::Publisher pub_hand_qbh1;
ros::Publisher pub_hand_qbh2;

struct traj_struct_joints{
	Eigen::Matrix<double, 7, 1> pos;
	Eigen::Matrix<double, 7, 1> vel;
	Eigen::Matrix<double, 7, 1> acc;
};
traj_struct_joints traj_joints;

struct traj_struct_cartesian{
	Eigen::Vector3d pos;
	Eigen::Vector3d vel;
	Eigen::Vector3d acc;
};
traj_struct_cartesian traj_cartesian;

struct UserData {
    std::vector<double> H;
    int l;
	double t;
} udata;

// ----- FUNCTIONS ----- //
void qbhand1_move(float);
void qbhand2_move(float, float);
double objective(const std::vector<double> &x, std::vector<double> &grad, void *data);
double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l);

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum){
	cout << "Caught signal " << signum << endl;
	// Terminate program
	exit(signum);
}

// void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
//   pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//   orient << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
// }

void jointsCallback( const sensor_msgs::JointStateConstPtr& msg ){
	q = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
	init_q = true;
}

void udataCallback(const panda_controllers::udata::ConstPtr& msg){

	for(int i = 0; i<70; ++i){
		H_vec.segment(i*PARAM, PARAM) << msg->H_stack[i*PARAM], msg->H_stack[i*PARAM+1], msg->H_stack[i*PARAM+2], msg->H_stack[i*PARAM+3], msg->H_stack[i*PARAM+4], msg->H_stack[i*PARAM+5], msg->H_stack[i*PARAM+6], msg->H_stack[i*PARAM+7], msg->H_stack[i*PARAM+8], msg->H_stack[i*PARAM+9];
	}

}

// void interpolator_joints(Eigen::Matrix<double, 7, 1> pos_i, Eigen::Matrix<double, 7, 1> pos_f, double tf, double t){
// 	traj_joints.pos << pos_i + (pos_i - pos_f)*(15*pow((t/tf),4) - 6*pow((t/tf),5) -10*pow((t/tf),3));
// 	traj_joints.vel << (pos_i - pos_f)*(60*(pow(t,3)/pow(tf,4)) - 30*(pow(t,4)/pow(tf,5)) -30*(pow(t,2)/pow(tf,3)));
// 	traj_joints.acc << (pos_i - pos_f)*(180*(pow(t,2)/pow(tf,4)) - 120*(pow(t,3)/pow(tf,5)) -60*(t/pow(tf,3)));
// }

traj_struct_cartesian interpolator_cartesian(Eigen::Vector3d pos_i, Eigen::Vector3d d_pos_i, Eigen::Vector3d dd_pos_i, Eigen::Vector3d pos_f, Eigen::Vector3d d_pos_f, Eigen::Vector3d dd_pos_f, double tf, double t){
	traj_struct_cartesian traj;
	traj.pos = min_jerk.get_q(pos_i, d_pos_i, dd_pos_i, pos_f, d_pos_f, dd_pos_f, tf, t);
	traj.vel = min_jerk.get_dq(pos_i, d_pos_i, dd_pos_i, pos_f, d_pos_f, dd_pos_f, tf, t);
	traj.acc = min_jerk.get_ddq(pos_i, d_pos_i, dd_pos_i, pos_f, d_pos_f, dd_pos_f, tf, t);
	return traj;
}

traj_struct_cartesian estimate_traj(Eigen::Vector3d center, double tf, double t){
	traj_struct_cartesian traj;
	double x0,y0,z0;
	double A = ampX, B = ampY, C = ampZ;
	double a = freqX, b = freqY, c = freqZ;
	double dx = phiX, dz = phiZ;
	double t0 = 1.0/(4*b);
	x0 = center(0) + offX;
	y0 = center(1) + offY;
	z0 = center(2) + offZ;

	traj.pos << x0 + A * std::sin(2*M_PI * a * (t-t0) + dx),
		y0 + B * std::cos(2*M_PI * b * (t-t0)),
		z0 + C * std::sin(2*M_PI * c * (t-t0) + dz);
	traj.vel << 2*M_PI *A * a * std::cos(2*M_PI * a * (t-t0) + dx),
		-2*M_PI *B * b * std::sin(2*M_PI * b * (t-t0)),
		2*M_PI *C * c * std::cos(2*M_PI * c * (t-t0) + dz);
	traj.acc << -2*M_PI *2*M_PI *A * a * a * std::sin(2*M_PI * a * (t-t0) + dx),
		-2*M_PI *2*M_PI *B * b * b * std::cos(2*M_PI * b * (t-t0)), 
		-2*M_PI *2*M_PI *C * c * c * std::sin(2*M_PI * c * (t-t0) + dz);
	return traj;
}

void frankaCallback(const franka_msgs::FrankaStateConstPtr& msg){
	Eigen::Matrix3d rotation;
	rotation << msg->O_T_EE[0], msg->O_T_EE[4], msg->O_T_EE[8],
				msg->O_T_EE[1], msg->O_T_EE[5], msg->O_T_EE[9],
				msg->O_T_EE[2], msg->O_T_EE[6], msg->O_T_EE[10];
	Eigen::Quaterniond quaternion(rotation);
	quaternion.normalize();
	p << msg->O_T_EE[12], msg->O_T_EE[13], msg->O_T_EE[14];
	Eigen::Vector4d coeffs = quaternion.coeffs();
	orient(0) = coeffs(0);
	orient(1) = coeffs(1);
	orient(2) = coeffs(2);
	orient(3) = coeffs(3);
	// q = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->q).data());
	init_p = true;
	if (!ready){
		p_start = p;
		p_end = p;
		ready = true;
	}
}

/*Funzione obiettivo problema di ottimo*/
double objective(const std::vector<double> &x, std::vector<double> &grad, void *data){

    UserData *udata = reinterpret_cast<UserData*>(data);
    Eigen::MatrixXd H_true;
    Eigen::VectorXd q(7);
    Eigen::VectorXd dq(7);
    Eigen::VectorXd ddq(7);
    int l;
    double t;

    H_true.resize(10,70);
    l = udata->l;
    t = udata->t;
	q_c << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0;

	if (!grad.empty()) {
        for (int i = 0; i < NJ; i++) {
            grad[i] = 0.0;
        }
    }

	/*Traiettoria sinusoidale ottima*/
    for(int i = 0; i < NJ; ++i){
            q(i) = q_c(i) + 0.30*sin(x[i]*t);     
            dq(i) = x[i]*0.30*cos(x[i]*t);
            ddq(i) = -x[i]*x[i]*0.30*sin(x[i]*t);      
    }
    
    for(int i=0; i<70; ++i){
        H_true.block(0,i,PARAM,1) << udata->H[i*PARAM], udata->H[i*PARAM+1], udata->H[i*PARAM+2], udata->H[i*PARAM+3], udata->H[i*PARAM+4], udata->H[i*PARAM+5], udata->H[i*PARAM+6], udata->H[i*PARAM+7], udata->H[i*PARAM+8], udata->H[i*PARAM+9];
    }
	
    fastRegMat.setArguments(q, dq, dq, ddq);
    Eigen::Matrix<double, NJ,PARAM*NJ> Y = fastRegMat.getReg();
    Eigen::Matrix<double, NJ,PARAM> redY = Y.block(0,(NJ-1)*PARAM,NJ,PARAM);

    return -redStackCompute(redY, H_true, l);
}

double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l){
    const int P = 10;
    const double epsilon = 0.1;
    double Vmax = 0;

    if (l <= (P-1)){
        if ((red_Y.transpose()-H.block(0,l*NJ,P,NJ)).norm()/(red_Y.transpose()).norm() >= epsilon){
            H.block(0,l*NJ,P,NJ) = red_Y.transpose();
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


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_throw");
	ros::NodeHandle node_handle;

	// std::string robot_name = ROBOT_NAME;
	float RATE;
	int SIMULATION;
	int GRIPPER;
	float TF_THROW;
	float TF_BRAKE;
	float TF_EST_BRAKE;
	float TF_EST;
	float HAND_DELAY;
	float TF_THROW_EST;
	std::vector<double> Q0_THROW;
	std::vector<double> P0_THROW;
	std::vector<double> P0_EST;
	std::vector<double> QF_THROW;
	std::vector<double> PF_THROW;
	std::vector<double> DPF_THROW;
	std::vector<double> P0_THROW_EST;
	std::vector<double> PF_THROW_EST;
	std::vector<double> DPF_THROW_EST;

	q_c << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0;

	if(!node_handle.getParam("/throw_node/SIMULATION", SIMULATION))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/RATE", RATE))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/GRIPPER", GRIPPER))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/TF_THROW", TF_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/TF_BRAKE", TF_BRAKE))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/TF_EST_BRAKE", TF_EST_BRAKE))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/HAND_DELAY", HAND_DELAY))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/Q0_THROW", Q0_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/QF_THROW", QF_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/P0_THROW", P0_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/P0_EST", P0_EST))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/PF_THROW", PF_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/DPF_THROW", DPF_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/TF_EST", TF_EST))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/P0_THROW_EST", P0_THROW_EST))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/TF_THROW_EST", TF_THROW_EST))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/PF_THROW_EST", PF_THROW_EST))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/DPF_THROW_EST", DPF_THROW_EST))
		ROS_ERROR("Failed to get parameter from server.");
	if (!node_handle.getParam("lissajous/ampX", ampX) ||
			!node_handle.getParam("lissajous/ampY", ampY) ||
			!node_handle.getParam("lissajous/ampZ", ampZ) ||
			!node_handle.getParam("lissajous/freqX", freqX) ||
			!node_handle.getParam("lissajous/freqY", freqY) ||
			!node_handle.getParam("lissajous/freqZ", freqZ) ||
			!node_handle.getParam("lissajous/phiX", phiX) ||
			!node_handle.getParam("lissajous/phiZ", phiZ) ||
			!node_handle.getParam("lissajous/offX", offX) ||
			!node_handle.getParam("lissajous/offY", offY) ||
			!node_handle.getParam("lissajous/offZ", offZ) ||
			!node_handle.getParam("lissajous/duration", liss_T)) {
		ROS_ERROR("Could not get trajectory parameters!");
		return false;
	}

	if (SIMULATION){
		robot_name = "";
	}else{
		robot_name = "";
		// robot_name = "/robot/arm";
	}

	// ----- Subscriber and Publishers ----- //
	// ros::Publisher pub_traj = node_handle.advertise<panda_controllers::Commands>(robot_name + "/computed_torque_controller/command", 1000);
	ros::Publisher pub_traj_cartesian = node_handle.advertise<panda_controllers::desTrajEE>("/CT_mod_controller_OS/command_cartesian", 1);
	ros::Subscriber sub_joints =  node_handle.subscribe<sensor_msgs::JointState>(robot_name + "/franka_state_controller/joint_states", 1, &jointsCallback);
	ros::Subscriber sub_franka = node_handle.subscribe<franka_msgs::FrankaState>(robot_name + "/franka_state_controller/franka_states", 1, &frankaCallback);
	/*Sub per calcolo traiettoria ottima*/
	ros::Subscriber sub_config_opt = node_handle.subscribe<panda_controllers::udata>("/CT_mod_controller_OS/opt_data", 1, &udataCallback);
	pub_hand_qbh1 = node_handle.advertise<trajectory_msgs::JointTrajectory>("/robot/gripper/qbhand1/control/qbhand1_synergy_trajectory_controller/command", 1);
	pub_hand_qbh2 = node_handle.advertise<trajectory_msgs::JointTrajectory>("/robot/gripper/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command", 1);
	// ros::Subscriber sub_pose =  node_handle.subscribe("/franka_state_controller/franka_ee_pose", 1, &poseCallback);
	ros::Publisher pub_cmd_opt = node_handle.advertise<sensor_msgs::JointState>("/CT_mod_controller_OS/command_joints_opt", 1);
	ros::Publisher pub_flagAdaptive = node_handle.advertise<panda_controllers::flag>("/CT_mod_controller_OS/adaptiveFlag", 1);
	ros::Publisher pub_flag_opt = node_handle.advertise<panda_controllers::flag>("/CT_mod_controller_OS/optFlag", 1);

	// ----- Messages ----- //
	panda_controllers::flag adaptive_flag_msg;
	panda_controllers::flag opt_flag_msg;
	panda_controllers::desTrajEE traj_msg;
	sensor_msgs::JointState traj_opt_msg;

	/*Resizie*/
	traj_opt_msg.position.resize(NJ);
    traj_opt_msg.velocity.resize(NJ);
    traj_opt_msg.effort.resize(NJ);

	// creating trajectory message
	// panda_controllers::Commands command_msg;
	// std_msgs::Int32 throw_flag_msg;
	// geometry_msgs::PoseStamped pos_des_msg;

	srand(time(NULL));
	bool first_time = true;
	double tf = 3.0;
	double rate = RATE;
	double tf_throw = TF_THROW;
	double tf_throw_est = TF_THROW_EST;
	double tf_brake = TF_BRAKE;
	double tf_0 = 3.0;
	double tf_est = TF_EST;
	double tf_est_brake = TF_EST_BRAKE;
	double t_new = 0.0;
	// Eigen::Matrix<double, 7, 1> qf;
	// Eigen::Vector3d pf;
	Eigen::Vector3d zero;
	zero.setZero();
	Eigen::Matrix<double, 7, 1> q0_throw = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(Q0_THROW.data(), Q0_THROW.size());
	Eigen::Vector3d p0_throw = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(P0_THROW.data(), P0_THROW.size());
	Eigen::Matrix<double, 7, 1> qf_throw = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(QF_THROW.data(), QF_THROW.size());
	Eigen::Vector3d pf_throw = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(PF_THROW.data(), PF_THROW.size());
	Eigen::Vector3d p0_est= Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(P0_EST.data(), P0_EST.size());
	Eigen::Vector3d dpf_throw = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(DPF_THROW.data(), DPF_THROW.size());
	Eigen::Vector3d p0_throw_est = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(P0_THROW_EST.data(), P0_THROW_EST.size());
	Eigen::Vector3d pf_throw_est = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(PF_THROW_EST.data(), PF_THROW_EST.size());
	Eigen::Vector3d dpf_throw_est = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(DPF_THROW_EST.data(), DPF_THROW_EST.size());
	Eigen::Vector3d pf_brake;
	Eigen::Vector3d pf_brake_est;
	Eigen::Vector3d p_center;	// starting point of estimating trajectory
	Eigen::Vector3d p_saved = p0_est;	// starting point of estimating trajectory
	// Eigen::Vector3d p_est_end;	// stopping point of estimating trajectory
	// Eigen::Vector3d vel_est_end;	// used to save ending velocity of estimating trajectory
	// Eigen::Vector3d vel_est_start;	// used to save starting velocity of estimating trajectory
	// Eigen::Vector3d vel_est_end;	// used to save ending velocity of estimating trajectory
	traj_struct_cartesian traj_est_start;
	traj_struct_cartesian traj_est_end;
	traj_struct_cartesian traj_tmp;
	XmlRpc::XmlRpcValue menu_par;

	// Initialize Ctrl-C
	signal(SIGINT, signal_callback_handler);
	// SET SLEEP TIME 1000 ---> 1 kHz
	ros::Rate loop_rate(RATE);

	const double q_lim_upp[] = {1.7, 1.5, 2.5, -0.1, 2.7, 3.5, 2.7};
	const double q_lim_low[] = {-1.7, -1.0, -2.5, -2.9, -2.7, 0.1, -2.7};
	double q_center[7];
	for (int i=0; i<7; i++){
		q_center[i] = (q_lim_upp[i] + q_lim_low[i])/2;
	}
	ros::Time t_init;
	init_q = false;
	init_p = false;
	ready = false;
	for(int i=0; i<7; i++){
		double q_low = q_lim_low[i];
		double q_upp = q_lim_upp[i];
		q0_throw(i) = (q_low + q_upp) / 2;
	}
	// double pos_des[3] = {1.2, 0.0, 0.0};
	double t = 0;
	int choice = 0;
	int choice_2 = 0;
	int executing = 0;

	/* Inizializzazione grandezze ottimo*/
	udata.H.resize(700);
	std::vector<double> lb(7), ub(7);
	std::vector<double> x(7);
	lb[0] = -M_PI; lb[1] = -M_PI; lb[2] = -M_PI_2; lb[3] = -M_PI; lb[4] = -M_PI_2; lb[5] = -M_PI_2; lb[6] = -M_PI_2;
	ub[0] = M_PI; ub[1] = M_PI; ub[2] = M_PI_2; ub[3] = M_PI; ub[4] = M_PI_2; ub[5] = M_PI_2; ub[6] = M_PI_2;

	while (ros::ok()){
		if (executing == 0){
			cout<<"choice:   (1: get pos,  2: set tf,  3: go to,  4: throw,  5: estimate,  6: adaptive,  7: reset pos) "<<endl;
			cin>>choice;
			while (!ready) ros::spinOnce();
			if (choice == 1){
				// --- save q, p --- //
				init_q = false;
				init_p = false;
				while((init_q==false) || (init_p==false)){
					ros::spinOnce();
				}
				p_saved = p;
				cout<<"q: ["<<q[0]<<", "<<q[1]<<", "<<q[2]<<", "<<q[3]<<", "<<q[4]<<", "<<q[5]<<", "<<q[6]<<"]"<<endl;
				cout<<"ee_pose: ["<<p[0]<<", "<<p[1]<<", "<<p[2]<<", "<<orient[0]<<", "<<orient[1]<<", "<<orient[2]<<", "<<orient[3]<<"]"<<endl;
				cout<<"saved!"<<endl;
			}else if (choice == 2){
				// --- set tf--- //
				cout<<"tf throwing: ";
				cin>>tf_throw;
				cout<<"tf estimate: ";
				cin>>tf_est;
			}else if (choice == 3){
				// --- go to --- //
				cout<<"where:   (1: p0_throw,  2: pf_throw,  3: p0_est,  4: p_saved,     0: cancel) "<<endl;
				cin>>choice_2;
				if (choice_2 == 0){
					choice = 0;
				}else {
					p_start = p_end;
					executing = 1;
					tf = tf_0;
					if (choice_2 == 1) 			p_end = p0_throw;
					else if (choice_2 == 2) 	p_end = pf_throw;
					else if (choice_2 == 3) 	p_end = p0_est;
					else if (choice_2 == 4) 	p_end = p_saved;
					else {
						executing = 0;
						p_end = p_start;
					}
				}
			}else if (choice == 4){
				// --- throw --- //
				first_time = true; // used for hand opening
				pf_brake = pf_throw + dpf_throw*tf_brake/2;
				p_start = p_end;
				p_end = pf_brake;
				executing = 2;
				tf = tf_throw + tf_brake;
			}else if (choice == 5){
				// --- estimate --- //
				cout<<"estimation type:   (1: lissajous,  2: min-jerk,  3:traj_opt ,0: cancel) "<<endl;
				cin>>choice_2;
				if (choice_2 == 1){
					// trajectory center
					p_center = p_end;
					// set executing
					executing = 3;
					// // starting adaptive
					// adaptive_flag_msg.header.stamp = ros::Time::now();
					// adaptive_flag_msg.flag = true;
					// pub_flagAdaptive.publish(adaptive_flag_msg);
					// trajectory stuff
					tf = tf_est + 1.0;
					// obtain estimate_traj velocity
					traj_est_start = estimate_traj(p_center, tf_est, tf_est_brake);
					traj_est_end = estimate_traj(p_center, tf_est, tf_est);
					traj_tmp = estimate_traj(p_center, tf_est, tf_est-tf_est_brake);
					// vel_est_end = traj_cartesian.vel;
					// acc_est_end = traj_cartesian.acc;
					p_end = traj_est_end.pos;
				} else if (choice_2 == 2){
					pf_brake_est = pf_throw_est + dpf_throw_est*tf_brake/2;
					p_start = p_end;
					p_end = pf_brake_est;
					executing = 4;
					tf = tf_0 + tf_throw_est + tf_brake;
				} else if (choice_2 == 3){
					executing = 5;
					tf = tf_est;
				}
			}else if (choice == 6){
				cout<<"adaptive:   (0: disable,  1: enable,     other: cancel) "<<endl;
				cin>>choice_2;
				if (choice_2 == 0){
					// stopping adaptive
					adaptive_flag_msg.header.stamp = ros::Time::now();
					adaptive_flag_msg.flag = false;
					pub_flagAdaptive.publish(adaptive_flag_msg);
					cout<<"adaptive disabled!"<<endl;
				} else if (choice_2 == 1){
					// starting adaptive
					adaptive_flag_msg.header.stamp = ros::Time::now();
					adaptive_flag_msg.flag = true;
					pub_flagAdaptive.publish(adaptive_flag_msg);
					cout<<"adaptive enabled!"<<endl;
				}
			}
			else if (choice == 7){
				ready = false;
			}
		}else{
			// ----- init trajectory cycle ----- //
			t_init = ros::Time::now();
			t = (ros::Time::now() - t_init).toSec();
			// ----- TRAJECTORY EXECUTION ----- //
			while (t <= tf){
				if (executing == 1){
					// --- go to --- //
					traj_cartesian = interpolator_cartesian(p_start, zero, zero, p_end, zero, zero, tf, t);
				}else if (executing == 2){
					// --- throwing --- //
					if (t <= tf_throw){
						traj_cartesian = interpolator_cartesian(p_start, zero, zero, pf_throw, dpf_throw, zero, tf_throw, t);
						if (t > tf_throw - HAND_DELAY){
							if (first_time){
								qbhand1_move(1.0);
								first_time = false;
							}
						}
					}else{
						traj_cartesian = interpolator_cartesian(pf_throw, dpf_throw, zero, pf_brake, zero, zero, tf_brake, t-tf_throw);
					}
				}else if (executing == 3){
					// --- Estimating --- //
					if (t < tf_est_brake){
						// - start estimating trajectory - //
						traj_cartesian = interpolator_cartesian(p_center, zero, zero, traj_est_start.pos, traj_est_start.vel, traj_est_start.acc, tf_est_brake, t);
					}else if ((t >= tf_est_brake) && (t < tf_est - tf_est_brake)){
						// - estimating - //
						// t_new = t - tf_est_brake;
						t_new = t;
						traj_cartesian = estimate_traj(p_center, tf_est, t_new);
					}else if ((t >= tf_est-tf_est_brake) && (t < tf_est)){
						// - braking estimate trajectory - //
						t_new = t - (tf_est-tf_est_brake);
						traj_cartesian = interpolator_cartesian(traj_tmp.pos, traj_tmp.vel, traj_tmp.acc, traj_est_end.pos, zero, zero, tf_est_brake, t_new);
					}else if ((t >= tf_est) && (t <= tf_est + 1.0)){
						// - stay in p_end - //
						t_new = t - tf_est;
						traj_cartesian.pos = traj_est_end.pos;
						traj_cartesian.vel = zero;
						traj_cartesian.acc = zero;
					}else{
						break;
					}
				}else if (executing == 4){
					// --- Estimating with throw --- //
					if (t <= tf_0){
						traj_cartesian = interpolator_cartesian(p_start, zero, zero, p0_throw_est, zero, zero, tf_0, t);
					}else if ((t > tf_0) && (t <= tf_0 + tf_throw_est)){
						// traj_cartesian = interpolator_cartesian(pf_throw_est, dpf_throw_est, zero, pf_brake_est, zero, zero, tf_brake, t-tf_throw_est);
						traj_cartesian = interpolator_cartesian(p0_throw_est, zero, zero, pf_throw_est, dpf_throw_est, zero, tf_throw_est, t-tf_0);
					}else if ((t > tf_0 + tf_throw_est) && (t <= tf_0 + tf_throw_est + tf_brake)){
						traj_cartesian = interpolator_cartesian(pf_throw_est, dpf_throw_est, zero, pf_brake_est, zero, zero, tf_brake, t-tf_0-tf_throw_est);
					} else {
						traj_cartesian.pos = pf_brake_est;
						traj_cartesian.vel = zero;
						traj_cartesian.acc = zero;
					}
				}else if (executing == 5){
					ros::spinOnce();

					for(int i = 0; i < 700; ++i){
						udata.H[i] = H_vec(i);
					}
					// double t = (ros::Time::now() - t_init).toSec();
					udata.t = t;
					traj_joints.pos.setZero();
					traj_joints.vel.setZero();
					traj_joints.acc.setZero();

					for(int i=0; i < 7; ++i){
            			x[i] = 0;
					}
					udata.l = 11;
					nlopt::opt opt(nlopt::algorithm::LN_COBYLA, NJ);

					opt.set_xtol_rel(1e-4);
					opt.set_lower_bounds(lb); // setto limite inferiore
					opt.set_upper_bounds(ub); // setto limite superiore 
					opt.set_min_objective(objective, &udata);  // definisco costo da massimizzare
				

					// Ottimizzazione
					double minf;
					nlopt::result result = opt.optimize(x, minf);

					/*Traiettoria ottima sinusoidale ottenuta*/
					for(int i = 0; i < 7; ++i){
						traj_joints.pos(i) = q_c(i) + 0.30*sin(x[i]*t);     
						traj_joints.vel(i) = x[i]*0.30*cos(x[i]*t);
						traj_joints.acc(i) = -x[i]*x[i]*0.30*sin(x[i]*t);      
        			}
				}
			

				// ----- publishing ----- //
				if (executing == 5){
					// cout << "traj:"<< traj_joints.pos;
					for(int i=0;i<NJ;i++){
						traj_opt_msg.header.stamp = ros::Time::now();
						traj_opt_msg.position[i] = traj_joints.pos(i);
						traj_opt_msg.velocity[i] = traj_joints.vel(i);
						traj_opt_msg.effort[i] = traj_joints.acc(i);
					}
					
					opt_flag_msg.flag = true;
					pub_flag_opt.publish(opt_flag_msg);
					pub_cmd_opt.publish(traj_opt_msg);
				}else{	
					traj_msg.header.stamp = ros::Time::now();
					traj_msg.position.x = traj_cartesian.pos(0);
					traj_msg.position.y = traj_cartesian.pos(1);
					traj_msg.position.z = traj_cartesian.pos(2);
					traj_msg.velocity.x = traj_cartesian.vel(0);
					traj_msg.velocity.y = traj_cartesian.vel(1);
					traj_msg.velocity.z = traj_cartesian.vel(2);
					traj_msg.acceleration.x = traj_cartesian.acc(0);
					traj_msg.acceleration.y = traj_cartesian.acc(1);
					traj_msg.acceleration.z = traj_cartesian.acc(2);
					
					opt_flag_msg.flag = false;
					pub_flag_opt.publish(opt_flag_msg);
					pub_traj_cartesian.publish(traj_msg);  
				}	
				loop_rate.sleep();
				t = (ros::Time::now() - t_init).toSec();
			}
			executing = 0;
			opt_flag_msg.flag = false;
			pub_flag_opt.publish(opt_flag_msg);
			// reset first_time
			first_time = true;
		}
		loop_rate.sleep();
	}
	return 0;
}

/*Function for closing and opening the softhands */
void qbhand2_move(float synergy, float manipulation){
	trajectory_msgs::JointTrajectory joint_traj_msg;

	// Header
	joint_traj_msg.header.stamp = ros::Time::now();
	joint_traj_msg.header.seq = 0;
	joint_traj_msg.header.stamp.sec = 0;
	joint_traj_msg.header.stamp.nsec = 0;

	// Define Joint Names
	joint_traj_msg.joint_names = {"qbhand2m1_manipulation_joint", "qbhand2m1_synergy_joint"};

	// Positions, Velocities, Accelerations, Effort and time_from_start
	trajectory_msgs::JointTrajectoryPoint joint_traj_point_msg;

	joint_traj_point_msg.positions = {synergy, manipulation};
	joint_traj_point_msg.velocities = {0, 0};
	joint_traj_point_msg.accelerations = {0, 0};
	joint_traj_point_msg.effort = {0, 0};
	joint_traj_point_msg.time_from_start.sec = 1;

	// Assign
	joint_traj_msg.points.push_back(joint_traj_point_msg);

	// Publish the message to close the hand
	pub_hand_qbh2.publish(joint_traj_msg);

	ROS_INFO("Hand moved!");
}

void qbhand1_move(float synergy){

	trajectory_msgs::JointTrajectory joint_traj_msg;

	// Header
	joint_traj_msg.header.stamp = ros::Time::now();
	joint_traj_msg.header.seq = 0;
	joint_traj_msg.header.stamp.sec = 0;
	joint_traj_msg.header.stamp.nsec = 0;

	// Define Joint Names
	joint_traj_msg.joint_names = {"qbhand1_synergy_joint"};

	// Positions, Velocities, Accelerations, Effort and time_from_start
	trajectory_msgs::JointTrajectoryPoint joint_traj_point_msg;

	joint_traj_point_msg.positions = {synergy};
	joint_traj_point_msg.velocities = {0};
	joint_traj_point_msg.accelerations = {0};
	joint_traj_point_msg.effort = {0};
	joint_traj_point_msg.time_from_start.sec = 1;

	// Assign
	joint_traj_msg.points.push_back(joint_traj_point_msg);

	// Publish the message to close the hand
	pub_hand_qbh1.publish(joint_traj_msg);
	ROS_INFO("Softhand1 moved!");
}