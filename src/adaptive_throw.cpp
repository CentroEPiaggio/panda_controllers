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
#include "panda_controllers/rpy.h"
#include "panda_controllers/impedanceGain.h"

#include "ros/ros.h"
// #include "panda_controllers/CommandParams.h"
// #include "panda_controllers/Commands.h"

#include <sstream>

/*Libreria per ottimo*/
#include "utils/thunder_panda_2.h"
#include "nlopt.hpp"
#include "utils/utils_cartesian.h"

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
// #include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "franka_msgs/FrankaState.h"
#include "franka_msgs/ErrorRecoveryActionGoal.h"
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

bool ready;
bool ready2;
bool init_q = false;
bool init_p = false;
std:: string robot_name;

Eigen::Matrix<double, 7, 1> q;
Eigen::Matrix<double, 7, 1> q_end;
Eigen::Matrix<double, 3, 1> ee_vel;
Eigen::Matrix<double, 7, 1> q_start;
Eigen::Matrix<double, 7, 1> q_saved;

Eigen::Vector3d p;
Eigen::Vector3d pose;
Eigen::Matrix<double, 4, 1> orient;
Eigen::Vector3d p_start;	// used to save starting point of tragectory (go to start)
Eigen::Vector3d p_end;	// used to save starting point of tragectory (go to start)
Eigen::Vector3d pose_start;	// used to save starting point of tragectory (go to start)
Eigen::Vector3d pose_end;	// used to save starting orientation of EE(go to start)		

Eigen::Quaterniond q_interpolated;
Eigen::Vector3d angular_velocity;
min_jerk_class min_jerk;

// lissagious parameters
double ampX, ampY, ampZ, freqX, freqY, freqZ, phiX, phiZ, offX, offY, offZ, liss_T;

/*Opt variable*/
// Eigen::MatrixXd H_true;
// Eigen::VectorXd S(10);
int counter;
Eigen::Matrix<double, NJ,PARAM*NJ> Y; 
Eigen::Matrix<double, NJ,PARAM> redY;
Eigen::VectorXd H_vec(700);
int l_counter;
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
traj_struct_joints traj_joints_old;

struct traj_struct_cartesian{
	Eigen::Vector3d pos;
	Eigen::Vector3d vel;
	Eigen::Vector3d acc;
};
traj_struct_cartesian traj_cartesian;

struct pose_struct_cartesian{
	Eigen::Vector3d rpy;
	Eigen::Vector3d angular_vel;
	Eigen::Vector3d angular_acc;
};
pose_struct_cartesian pose_cartesian;


struct UserData {
    std::vector<double> H;
    int l;
	double t;
	int count;
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
	if (!ready2){
		q_start = q;
		q_end = q_start;
		ready2 = true;
	}
}

void udataCallback(const panda_controllers::udata::ConstPtr& msg){

	for(int i = 0; i<70; ++i){
		H_vec.segment(i*PARAM, PARAM) << msg->H_stack[i*PARAM], msg->H_stack[i*PARAM+1], msg->H_stack[i*PARAM+2], msg->H_stack[i*PARAM+3], msg->H_stack[i*PARAM+4], msg->H_stack[i*PARAM+5], msg->H_stack[i*PARAM+6], msg->H_stack[i*PARAM+7], msg->H_stack[i*PARAM+8], msg->H_stack[i*PARAM+9];
	}
	counter = msg->count;
	l_counter = msg->l;
	for(int i = 0; i<7; ++i){
		ee_vel(i) = msg->ee_velocity[i];
	// cout << l_counter << endl;
	}

}

traj_struct_joints interpolator_joints(Eigen::VectorXd pos_i, Eigen::VectorXd d_pos_i, Eigen::VectorXd dd_pos_i, Eigen::VectorXd pos_f, Eigen::VectorXd d_pos_f, Eigen::VectorXd dd_pos_f, double tf, double t){
	traj_struct_joints traj;
	traj.pos = min_jerk.get_q(pos_i, d_pos_i, dd_pos_i, pos_f, d_pos_f, dd_pos_f, tf, t);
	traj.vel = min_jerk.get_dq(pos_i, d_pos_i, dd_pos_i, pos_f, d_pos_f, dd_pos_f, tf, t);
	traj.acc = min_jerk.get_ddq(pos_i, d_pos_i, dd_pos_i, pos_f, d_pos_f, dd_pos_f, tf, t);
	return traj;
}

traj_struct_cartesian interpolator_cartesian(Eigen::Vector3d pos_i, Eigen::Vector3d d_pos_i, Eigen::Vector3d dd_pos_i, Eigen::Vector3d pos_f, Eigen::Vector3d d_pos_f, Eigen::Vector3d dd_pos_f, double tf, double t){
	traj_struct_cartesian traj;
	traj.pos = min_jerk.get_q(pos_i, d_pos_i, dd_pos_i, pos_f, d_pos_f, dd_pos_f, tf, t);
	traj.vel = min_jerk.get_dq(pos_i, d_pos_i, dd_pos_i, pos_f, d_pos_f, dd_pos_f, tf, t);
	traj.acc = min_jerk.get_ddq(pos_i, d_pos_i, dd_pos_i, pos_f, d_pos_f, dd_pos_f, tf, t);
	return traj;
}

// Funzione per convertire angoli RPY in quaternioni(Ordine rotazione roll-pitch.yaw)
Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw, bool transf) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = rollAngle*pitchAngle*yawAngle;
	q.normalize();
    return q;
}

pose_struct_cartesian slerp(const Eigen::Vector3d& rpy_current, const Eigen::Vector3d& rpy_desired, double t, double t_f){
	
	// Convertire gli angoli Eulero in quaternioni
    Eigen::Quaterniond q_current = rpyToQuaternion(rpy_current[0], rpy_current[1], rpy_current[2], false);
    Eigen::Quaterniond q_desired = rpyToQuaternion(rpy_desired[0], rpy_desired[1], rpy_desired[2], true);

    // Calcolare il fattore di interpolazione normalizzato
    double t_normalized = t / t_f;

	// calcolo theta
	// double theta = acos(q_current.dot(q_desired));

	// setto vecchio quaternione di comando
	Eigen::Quaterniond q_interpolated_old = q_interpolated;
    
	// Calcolare il quaternione interpolato usando SLERP
    q_interpolated = q_current.slerp(t_normalized, q_desired);
	
	// Eigen::Quaterniond dq_interpolated;
	// if ((q_current*q_desired).w() >= 0){
	// 	Eigen::Quaterniond dq_interpolated = q_current.coeffs();
	// }

	// Differenza quaternione
	Eigen::Quaterniond delta_q = q_interpolated*q_interpolated_old.conjugate();
	Eigen::AngleAxisd angle_axis(delta_q);

	// velocità angolare
	Eigen::Vector3d angular_velocity_old = angular_velocity;
	angular_velocity = angle_axis.axis() * angle_axis.angle() / 0.001;

	// accelerazione angolare commandata
	Eigen::Vector3d angular_acceleration = (angular_velocity - angular_velocity_old) / 0.001;
    
    // Convertire il quaternione interpolato in angoli di Eulero
    Eigen::Vector3d euler_angles = q_interpolated.toRotationMatrix().eulerAngles(0, 1, 2);

	// cout << euler_angles << endl;
    pose_struct_cartesian com;
	com.rpy = euler_angles;
	com.angular_vel = angular_velocity; 
	com.angular_acc = angular_acceleration;
    
	return com;
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
	pose = rotation.eulerAngles(0,1,2);
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
		pose_start = pose;
		cout << pose_start << endl;
		pose_end = pose_start;
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
	int count;

    H_true.resize(10,70);
    l = udata->l;
    t = udata->t;
	count = udata->count;
	q_c << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0;

	// if (!grad.empty()) {
    //     for (int i = 0; i < NJ; i++) {
    //         grad[i] = 0.0;
    //     }
    // }

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

    return redStackCompute(redY, H_true, l);
}

double redStackCompute(const Eigen::Matrix<double, NJ, PARAM>& red_Y, Eigen::MatrixXd& H,int& l){
    const int P = 10;
    const double epsilon = 0.1;
    double Vmax;

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
	Eigen::JacobiSVD<Eigen::Matrix<double, PARAM, PARAM>> solver_cond(H*H.transpose());
	double lmax = (solver_cond.singularValues()).maxCoeff();
    return (lmax/Vmax);
	// return -Vmax;
}

    Eigen::Affine3d computeT0EE(const Eigen::VectorXd& q){
       
        Eigen::Matrix<double, NJ, 4> DH; // matrice D-H
        Eigen::Affine3d T0i = Eigen::Affine3d::Identity();
        Eigen::Affine3d T0n; 
        Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

        // Riempio sezione distanza "a"
        DH.block(0,0,NJ,1) << 0, 0, 0,0.0825, -0.0825, 0, 0.088;   
        // Riempio sezione angolo "alpha"
        DH.block(0,1,NJ,1) << 0, -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, M_PI_2, M_PI_2;
        // Riempio sezione distanza "d"
        DH.block(0,2,NJ,1) << 0.3330, 0, 0.3160, 0, 0.384, 0, 0.107; // verificato che questi valori corrispondono a DH che usa il robot in simulazione
        // Riempio sezione angolo giunto "theta"
        DH.block(0,3,NJ,1) = q;     

        for (int i = 0; i < NJ; ++i)
        {
            double a_i = DH(i,0);
            double alpha_i = DH(i,1);
            double d_i = DH(i,2);
            double q_i = DH(i,3);

            T << cos(q_i), -sin(q_i), 0, a_i,
                sin(q_i)*cos(alpha_i), cos(q_i)*cos(alpha_i), -sin(alpha_i), -sin(alpha_i)*d_i,
                sin(q_i)*sin(alpha_i), cos(q_i)*sin(alpha_i), cos(alpha_i), cos(alpha_i)*d_i,
                0, 0, 0, 1;

            // Avanzamento perice i 
            T0i.matrix() = T0i.matrix()*T;
        }
        T0n = T0i;
        /*If EE system differs from frame n(Like frame hand true robot)*/
        Eigen::Affine3d TnEE;
        Eigen::Vector3d dnEE;
        dnEE << 0.13, 0 , 0.035;
        T0n.translation() = T0i.translation() + T0i.linear()*dnEE;    
        
        return T0n;
    }


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_throw");
	ros::NodeHandle node_handle;
	bool start = true;
	bool smooth_flag = true;
	bool curr_vel_flag = false;
    int count = 0;
	double t_smooth = 0;
	Eigen::Matrix<double, 6, 1> Kp;
	Eigen::Matrix<double, 6, 1> Kv;

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
	std::vector<double> P0_PICK;
	std::vector<double> POSE0_PICK;
	std::vector<double> POSE0_THROW;
	std::vector<double> P0_THROW;
	std::vector<double> QF_THROW;
	std::vector<double> PF_THROW;
	std::vector<double> POSEF_THROW;
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
	if(!node_handle.getParam("/throw_node/P0_PICK", P0_PICK))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/P0_THROW", P0_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/POSE0_THROW", POSE0_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/POSE0_PICK", POSE0_PICK))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/POSEF_THROW", POSEF_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/PF_THROW", PF_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/DPF_THROW", DPF_THROW))
		ROS_ERROR("Failed to get parameter from server.");
	if(!node_handle.getParam("/throw_node/TF_EST", TF_EST))
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

	/*Inizializzo comando rpy*/
	pose_cartesian.rpy.setZero(); 
	pose_cartesian.angular_vel.setZero(); 
	pose_cartesian.angular_acc.setZero();

	// ----- Subscriber and Publishers ----- //
	// ros::Publisher pub_traj = node_handle.advertise<panda_controllers::Commands>(robot_name + "/computed_torque_controller/command", 1000);
	ros::Subscriber sub_joints =  node_handle.subscribe<sensor_msgs::JointState>(robot_name + "/franka_state_controller/joint_states", 1, &jointsCallback);
	ros::Subscriber sub_franka = node_handle.subscribe<franka_msgs::FrankaState>(robot_name + "/franka_state_controller/franka_states", 1, &frankaCallback);
	ros::Subscriber sub_config_opt = node_handle.subscribe<panda_controllers::udata>("/CT_mod_controller_OS/opt_data", 1, &udataCallback);
	// ros::Subscriber sub_config_opt = node_handle.subscribe<panda_controllers::udata>("/computed_torque_mod_controller/opt_data", 1, &udataCallback);
	// ros::Subscriber sub_pose =  node_handle.subscribe("/franka_state_controller/franka_ee_pose", 1, &poseCallback);
	pub_hand_qbh1 = node_handle.advertise<trajectory_msgs::JointTrajectory>("/robot/gripper/qbhand1/control/qbhand1_synergy_trajectory_controller/command", 1);
	pub_hand_qbh2 = node_handle.advertise<trajectory_msgs::JointTrajectory>("/robot/gripper/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command", 1);

	ros::Publisher pub_error = node_handle.advertise<franka_msgs::ErrorRecoveryActionGoal>(robot_name + "/franka_control/error_recovery/goal", 1);
	ros::Publisher pub_traj_cartesian = node_handle.advertise<panda_controllers::desTrajEE>("/CT_mod_controller_OS/command_cartesian", 1);
	ros::Publisher pub_rpy = node_handle.advertise<panda_controllers::rpy>("/CT_mod_controller_OS/command_rpy", 1);
	ros::Publisher pub_cmd_opt = node_handle.advertise<sensor_msgs::JointState>("/CT_mod_controller_OS/command_joints_opt", 1);
	ros::Publisher pub_flagAdaptive = node_handle.advertise<panda_controllers::flag>("/CT_mod_controller_OS/adaptiveFlag", 1);
	ros::Publisher pub_impedanceGains = node_handle.advertise<panda_controllers::impedanceGain>("/CT_mod_controller_OS/impedanceGains", 1);
	// ros::Publisher pub_cmd_opt = node_handle.advertise<sensor_msgs::JointState>("/computed_torque_mod_controller/command_joints_opt", 1);
	// ros::Publisher pub_flagAdaptive = node_handle.advertise<panda_controllers::flag>("/computed_torque_mod_controller/adaptiveFlag", 1);
	ros::Publisher pub_flag_opt = node_handle.advertise<panda_controllers::flag>("/CT_mod_controller_OS/optFlag", 1);
	ros::Publisher pub_flag_resetAdp = node_handle.advertise<panda_controllers::flag>("/CT_mod_controller_OS/resetFlag", 1);

	// ----- Messages ----- //
	panda_controllers::flag adaptive_flag_msg;
	panda_controllers::flag opt_flag_msg;
	panda_controllers::flag newAdp_flag_msg;
	panda_controllers::desTrajEE traj_msg;
	panda_controllers::rpy rpy_msg;
	panda_controllers::impedanceGain gains_msg;
	sensor_msgs::JointState traj_opt_msg;

	/*Resizie*/
	opt_flag_msg.flag = false;
	traj_opt_msg.position.resize(NJ);
    traj_opt_msg.velocity.resize(NJ);
    traj_opt_msg.effort.resize(NJ);

	srand(time(NULL));
	bool first_time = true;
	bool joint_move = false;
	bool set_pos = false;
	bool set_rot = false;

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
	Eigen::Matrix<double, 7, 1> zero_j;
	zero.setZero();
	zero_j.setZero();
	Eigen::Matrix<double, 7, 1> q0_throw = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(Q0_THROW.data(), Q0_THROW.size());
	Eigen::Vector3d p0_pick = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(P0_PICK.data(), P0_PICK.size());
	Eigen::Vector3d pose0_pick = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(POSE0_PICK.data(), POSE0_PICK.size());
	Eigen::Matrix<double, 7, 1> qf_throw = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(QF_THROW.data(), QF_THROW.size());
	Eigen::Vector3d pf_throw = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(PF_THROW.data(), PF_THROW.size());
	Eigen::Vector3d posef_throw = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(POSEF_THROW.data(), POSEF_THROW.size());
	Eigen::Vector3d p0_throw= Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(P0_THROW.data(), P0_THROW.size());
	Eigen::Vector3d pose0_throw= Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(POSE0_THROW.data(), POSE0_THROW.size());
	Eigen::Vector3d dpf_throw = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(DPF_THROW.data(), DPF_THROW.size());
	// Eigen::Vector3d p0_throw_est = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(P0_THROW_EST.data(), P0_THROW_EST.size());
	Eigen::Vector3d p0_throw_est = p0_throw;
	Eigen::Vector3d pf_throw_est = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(PF_THROW_EST.data(), PF_THROW_EST.size());
	Eigen::Vector3d dpf_throw_est = Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(DPF_THROW_EST.data(), DPF_THROW_EST.size());
	Eigen::Vector3d pf_brake;
	Eigen::Vector3d pf_brake_est;
	Eigen::Vector3d p_center;	// starting point of estimating trajectory
	Eigen::Vector3d p_saved;	// starting point of desidered position
	Eigen::Vector3d pose_saved;	// starting point of desidered pose

	traj_struct_cartesian traj_est_start;
	traj_struct_cartesian traj_est_end;
	traj_struct_cartesian traj_tmp;
	XmlRpc::XmlRpcValue menu_par;

	Eigen::Vector3d offset1;
	offset1 << 0, 0, 0.30;
	Eigen::Vector3d offset2;
	offset2 << 0, 0, 0.10;

	// Initialize Ctrl-C
	signal(SIGINT, signal_callback_handler);
	// SET SLEEP TIME 1000 ---> 1 kHz
	ros::Rate loop_rate(RATE);


	const double q_lim_upp[] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
	const double q_lim_low[] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
	double q_center[7];
	for (int i=0; i<7; i++){
		q_center[i] = (q_lim_upp[i] + q_lim_low[i])/2;
		q_saved[i] = q_center[i];
	}
	ros::Time t_init;
	init_q = false;
	init_p = false;
	ready = false;
	ready2 = false;

	for(int i=0; i<7; i++){
		double q_low = q_lim_low[i];
		double q_upp = q_lim_upp[i];
		// q0_throw(i) = (q_low + q_upp) / 2;
	}
	// double pos_des[3] = {1.2, 0.0, 0.0};
	double t = 0;
	int choice = 0;
	int choice_2 = 0;
	int executing = 0;

	// /*Initial Gains*/
	// Kp << 60, 60, 60, 20, 20, 20;
	// Kv << 30, 30, 30, 2, 2, 2;

	/* Inizializzazione grandezze ottimo, ub e lb scelti per non sforare i limiti ed evitare la saturazione*/
	udata.H.resize(700);
	std::vector<double> lb(7), ub(7);
	std::vector<double> x(7), x_old(7);
	lb[0] = -M_PI_4; lb[1] = -M_PI_2; lb[2] = -M_PI_4; lb[3] = -M_PI_4; lb[4] = -M_PI_4; lb[5] = -M_PI_4; lb[6] = -M_PI_2;
	ub[0] = M_PI_4; ub[1] = M_PI_2; ub[2] = M_PI_4; ub[3] = M_PI_4; ub[4] = M_PI_4; ub[5] = M_PI_4; ub[6] = M_PI_2;

	for(int i=0; i < 7; ++i){
		x_old[i] = 0;
	}

	while (ros::ok()){
		if (executing == 0){
			cout<<"choice:   (0: set command,  1: get pos,  2: set tf,  3: go to,  4: throw,  5: estimate,  6: adaptive,  7: reset pos,  8: reset error,  9: close/open hand) "<<endl;
			cin>>choice;
			newAdp_flag_msg.flag = false;
			smooth_flag = true;
			while (!ready) {
				ros::spinOnce();
				traj_joints.pos = q_start;
				traj_joints.vel.setZero();
				traj_joints.acc.setZero();
			}
			if (choice == 0){
				cout<<"what:  (1: Position,  2: Orientation,  0: Cancel)"<<endl;
				cin>>choice_2;
				if (choice_2 == 1){
					cout<<"x:"; cin>>p_saved(0);
					cout<<"y:"; cin>>p_saved(1);
					cout<<"z:"; cin>>p_saved(2);
					set_pos = true;
				}else if (choice_2 == 2){
					cout<<"roll:"; cin>>pose_saved(0); 
					cout<<"pitch:"; cin>>pose_saved(1); 
					cout<<"yaw:"; cin>>pose_saved(2);
					set_rot = true;
				} else{
					executing = 0;
				}
			}
		
			if (choice == 1){
				// --- save q, p --- //
				init_q = false;
				init_p = false;
				while((init_q==false) && (init_p==false)){
					ros::spinOnce();
				}
				q_saved = q;
				p_saved = p;
				pose_saved = pose;
				set_pos = true;
				set_rot = true;

				cout<<"q: ["<<q[0]<<", "<<q[1]<<", "<<q[2]<<", "<<q[3]<<", "<<q[4]<<", "<<q[5]<<", "<<q[6]<<"]"<<endl;
				cout<<"ee_pose: ["<<p[0]<<", "<<p[1]<<", "<<p[2]<<", "<<orient[0]<<", "<<orient[1]<<", "<<orient[2]<<", "<<orient[3]<<"]"<<endl;
				cout<<"rpy:["<<pose[0]<<", "<<pose[1]<<", "<<pose[2]<<"]"<<endl;
				cout<<"saved!"<<endl;
			}else if (choice == 2){
				// --- set tf--- //
				cout<<"tf throwing: ";
				cin>>tf_throw;
				cout<<"tf estimate: ";
				cin>>tf_est;
			}else if (choice == 3){
				// --- go to --- //
				cout<<"where:   (1: p0_pick,  2: pf_throw,  3: p0_throw,  4: config_saved,   5: point_saved(relative),   6: point_saved(absolute),    7: Go up,     0: cancel) "<<endl;
				cin>>choice_2;
				if (choice_2 == 0){
					choice = 0;
				}else {
					p_start = p_end;
					pose_start = pose_end;
					/*Reset joint position*/
					ready2 = false;
					ros::spinOnce();

					/*If precendent move is a joint movement*/					
					if(joint_move){
						q_start = q_end;
						p_start = computeT0EE(q_start).translation();
						pose_start = (computeT0EE(q_start).linear()).eulerAngles(0,1,2);
						joint_move = false;
					}

					executing = 1;
					tf = tf_0;

					if (choice_2 == 1){
						p_end = p0_pick;
						pose_end = pose0_pick;
						tf = tf + 2;
						// opt_flag_msg.flag = true;
						// q_end = q0_throw;
						// joint_move = true;
						// pose_end = pose_start+pose_saved;
					}
					else if (choice_2 == 2){
						p_end = pf_throw;
						// pose_end = posef_throw;
						// pose_end = pose_start+pose_saved;
						// cout << pose_end << endl;
					}
					else if (choice_2 == 3) {
						p_end = p0_throw;
						pose_end = pose0_throw;
					}	
					else if (choice_2 == 4) {
						opt_flag_msg.flag = true;
						q_end = q_saved;
						joint_move = true;
					}
					else if (choice_2 == 5) {
						p_end = p_start + p_saved;
						pose_end = pose_start+pose_saved;
						p_saved.setZero();
						pose_saved.setZero();
					}
					else if (choice_2 == 6) {
						if (set_pos)
							p_end = p_saved;
							p_saved.setZero();
                            // cout << p_saved << endl;
						if (set_rot)
							pose_end = pose_saved;
							pose_saved.setZero();
                            // cout << pose_saved << endl;
					}
					else if (choice_2 == 7){
						p_end = p_start+offset1;
						// pose_end = posef_throw;
						// pose_end = pose_start+pose_saved;
						// cout << pose_end << endl;
					}
					else {
						executing = 0;
						p_end = p_start;
					}
				}
			}else if (choice == 4){
				// --- throw --- //
				first_time = true; // used for hand opening
				pf_brake = pf_throw + dpf_throw*tf_brake/2;
				if(joint_move){
					p_start = computeT0EE(q).translation();
					joint_move = false;
				}else{
					p_start = p_end;
				}
				curr_vel_flag = true;
				p_end = pf_brake;
				executing = 2;
				tf = tf_throw + tf_brake;

				/*High gains for throw*/
				//Set throw gains
				Kp << 120, 120, 120, 15, 15, 15;
				Kv << 25, 25, 25, 2, 2, 2;
				for(int i = 0; i<6; ++i){
					gains_msg.stiffness[i] = Kp(i); 
					gains_msg.damping[i] = Kv(i);
				}
				pub_impedanceGains.publish(gains_msg);
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
					opt_flag_msg.flag = true;
					joint_move = true;
					executing = 5;
					tf = tf_est + 1.0;
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
					newAdp_flag_msg.flag = true;
					pub_flagAdaptive.publish(adaptive_flag_msg);
					cout<<"adaptive enabled!"<<endl;
				}
				pub_flag_resetAdp.publish(newAdp_flag_msg); // publish of reset H and E
			}else if (choice == 7){
				ready = false;
			}else if (choice == 8){
				franka_msgs::ErrorRecoveryActionGoal error_msg;
				pub_error.publish(error_msg);
				ready = false;
			}else if (choice == 9){
			// - Gripper Control - //
				float value;
				cout<<"value: "<<endl;
				cin>>value;
				if (GRIPPER == 0){
					// dh3_ctrl(value);
				}else if (GRIPPER == 1){
					qbhand1_move(value);
				}else if (GRIPPER == 2){
					qbhand2_move(value,0);
				}
			}
		}else{
			//Set high gains
			Kp <<100, 100, 100, 20, 20, 20;
			Kv << 25, 25, 25, 2, 2, 2;
			for(int i = 0; i<6; ++i){
				gains_msg.stiffness[i] = Kp(i); 
				gains_msg.damping[i] = Kv(i);
			}
			pub_impedanceGains.publish(gains_msg);
			// ----- init trajectory cycle ----- //
			t_init = ros::Time::now();
			t = (ros::Time::now() - t_init).toSec();
			t_smooth = t + 0.010;
			// ----- TRAJECTORY EXECUTION ----- //
			while (t <= tf){
				if (executing == 1){
					// --- go to --- //
					if(t <= tf_0){ 
						if (choice_2 == 1){
							// //Set pick gains
							// Kp << 100, 100, 100, 30, 30, 30;
							// Kv << 20, 20, 20, 2, 2, 2;
							// for(int i = 0; i<6; ++i){
							// 	gains_msg.stiffness[i] = Kp(i); 
							// 	gains_msg.damping[i] = Kv(i);
							// }
							// pub_impedanceGains.publish(gains_msg);
							traj_cartesian = interpolator_cartesian(p_start, zero, zero, p_end+offset2, zero, zero, tf_0, t);
						}else{
							// //Set pick gains
							// Kp << 60, 60, 60, 20, 20, 20;
							// Kv << 15, 15, 15, 2, 2, 2;
							// for(int i = 0; i<6; ++i){
							// 	gains_msg.stiffness[i] = Kp(i); 
							// 	gains_msg.damping[i] = Kv(i);
							// }
							// pub_impedanceGains.publish(gains_msg);
							traj_cartesian = interpolator_cartesian(p_start, zero, zero, p_end, zero, zero, tf_0, t);
						}
						traj_joints = interpolator_joints(q_start, zero_j, zero_j, q_end, zero_j, zero_j, tf_0, t);
						/*ORIENTATION COMMAND SLERP*/
						pose_cartesian = slerp(pose_start, pose_end, t, tf);
						
					}else{
						traj_cartesian = interpolator_cartesian(p_end+offset2, zero, zero, p_end, zero, zero, tf-tf_0, t-tf_0);
					}
					// if(choice_2 == 1){
					// 	//Set pick gains
					// 	Kp << 100, 100, 100, 30, 30, 30;
					// 	Kv << 20, 20, 20, 2, 2, 2;
					// 	for(int i = 0; i<6; ++i){
					// 		gains_msg.stiffness[i] = Kp(i); 
					// 		gains_msg.damping[i] = Kv(i);
					// 	}
					// 	pub_impedanceGains.publish(gains_msg);
					// }
					// cout << "command:" << rpy.pitch << endl;
				}else if (executing == 2){
					// --- throwing --- //
					if (t <= tf_throw){
						traj_cartesian = interpolator_cartesian(p_start, zero, zero, pf_throw, dpf_throw, zero, tf_throw, t);
						// traj_cartesian = interpolator_cartesian(p_start, zero, zero, pf_throw, zero, zero, tf_throw+tf_brake, t);
						if (t > tf_throw - HAND_DELAY){
							if (first_time){
								qbhand1_move(0.0);
								first_time = false;
							}
						}
					}else{
						if(curr_vel_flag){
							curr_vel_flag = false;
							ros::spinOnce();
							pf_brake = pf_throw + ee_vel*tf_brake/2;
							// cout << pf_brake << endl;
						}
						traj_cartesian = interpolator_cartesian(pf_throw, dpf_throw, zero, pf_brake, zero, zero, tf_brake, t-tf_throw);
						// traj_cartesian = interpolator_cartesian(pf_throw, zero, zero, pf_brake, zero, zero, tf_throw+tf_brake, t);
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
					
					for(int i=0; i < 7; ++i){
						x[i] = x_old[i];
					}
					traj_joints_old = traj_joints;
					
					if (counter%10 == 0){
                        // count = 0;
                        ros::spinOnce();
                        for(int i = 0; i < 700; ++i){
                            udata.H[i] = H_vec(i);
                        }
                    // double t = (ros::Time::now() - t_init).toSec();
                        udata.t = t;
                        udata.l = l_counter;
                        nlopt::opt opt(nlopt::algorithm::LN_COBYLA, NJ);
						double minf;

                        opt.set_xtol_rel(1e-4);
                        opt.set_lower_bounds(lb); // setto limite inferiore
                        opt.set_upper_bounds(ub); // setto limite superiore 
                        opt.set_min_objective(objective, &udata);  // definisco costo da massimizzare
                    

                        // Ottimizzazione
						// if(!smooth_flag){
						nlopt::result result = opt.optimize(x, minf);
					
						for(int i=0; i < 7; ++i){
							x_old[i] = x[i];
						}
				
						for(int i = 0; i < 7; ++i){
							traj_joints.pos(i) = q_c(i) + 0.30*sin(x[i]*t);     
							traj_joints.vel(i) = x[i]*0.30*cos(x[i]*t);
							traj_joints.acc(i) = -x[i]*x[i]*0.30*sin(x[i]*t);      
						}
						// }	
						// t_smooth = t + 0.010;
						// cout << t_smooth <<endl;
						// cout << traj_joints.pos <<endl;
						// cout << traj_joints_old.pos <<endl;
					}
					/*Parte di smoothing ai giunti*/
					// traj_joints = interpolator_joints(traj_joints_old.pos, traj_joints_old.vel, traj_joints_old.acc, traj_joints.pos, traj_joints.vel, traj_joints.acc, t_smooth, t);
					// if(t <= 1.0+t_smooth){
					// 	traj_joints = interpolator_joints(traj_joints_old.pos, traj_joints_old.vel, traj_joints_old.acc, traj_joints.pos, traj_joints.vel, traj_joints.acc, 1.0+t_smooth, t);
					// }else{
						// for(int i = 0; i < 7; ++i){
						// 	traj_joints.pos(i) = q_c(i) + 0.30*sin(x[i]*t);     
						// 	traj_joints.vel(i) = x[i]*0.30*cos(x[i]*t);
						// 	traj_joints.acc(i) = -x[i]*x[i]*0.30*sin(x[i]*t);      
						// 	smooth_flag = false;
						// }
					// 	t_smooth = t;
					// }
					/*Traiettoria ottima sinusoidale ottenuta*/
                    for(int i = 0; i < 7; ++i){
                        traj_joints.pos(i) = q_c(i) + 0.30*sin(x[i]*t);     
                        traj_joints.vel(i) = x[i]*0.30*cos(x[i]*t);
                        traj_joints.acc(i) = -x[i]*x[i]*0.30*sin(x[i]*t);      
                    }
                    // count = count+1;
				}
			

				// ----- publishing ----- //
				if (opt_flag_msg.flag){
					// cout << "traj:"<< traj_joints.pos;
					for(int i=0;i<NJ;i++){
						traj_opt_msg.header.stamp = ros::Time::now();
						traj_opt_msg.position[i] = traj_joints.pos(i);
						traj_opt_msg.velocity[i] = traj_joints.vel(i);
						traj_opt_msg.effort[i] = traj_joints.acc(i);
					}
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

					rpy_msg.angle[0] = pose_cartesian.rpy(0); // roll
					rpy_msg.angle[1] = pose_cartesian.rpy(1); // pitch
					rpy_msg.angle[2] = pose_cartesian.rpy(2); //yaw
					// rpy_msg.omega[0] = pose_cartesian.angular_vel(0); 
					// rpy_msg.omega[1] = pose_cartesian.angular_vel(1); 
					// rpy_msg.omega[2] = pose_cartesian.angular_vel(2);
					// rpy_msg.alpha[0] = pose_cartesian.angular_acc(0); 
					// rpy_msg.alpha[1] = pose_cartesian.angular_acc(1); 
					// rpy_msg.alpha[2] = pose_cartesian.angular_acc(2);
					pub_rpy.publish(rpy_msg);
					pub_traj_cartesian.publish(traj_msg);  
				}	
				loop_rate.sleep();
				t = (ros::Time::now() - t_init).toSec();
				pub_flag_opt.publish(opt_flag_msg);
			}
			if (opt_flag_msg.flag){
				/*Command to stay*/
				ros::spinOnce();
				Eigen::Vector3d p_fin = (computeT0EE(q)).translation();
				traj_msg.position.x = p_fin(0);
				traj_msg.position.y = p_fin(1);
				traj_msg.position.z = p_fin(2);
				traj_msg.velocity.x = 0;
				traj_msg.velocity.y = 0;
				traj_msg.velocity.z = 0;
				traj_msg.acceleration.x = 0;
				traj_msg.acceleration.y = 0;
				traj_msg.acceleration.z = 0;
				pub_traj_cartesian.publish(traj_msg);  
			}
			// p_saved.setZero();
			// pose_saved.setZero();
			executing = 0;
			opt_flag_msg.flag = false;
			pub_flag_opt.publish(opt_flag_msg);
			// reset first_time
			first_time = true;

			/*Return to initial Gains*/
			// Kp << 60, 60, 60, 20, 20, 20;
			// Kv << 15, 15, 15, 2, 2, 2;
			// for(int i = 0; i<6; ++i){
			// 	gains_msg.stiffness[i] = Kp(i); 
			// 	gains_msg.damping[i] = Kv(i);
			// }
			// pub_impedanceGains.publish(gains_msg);
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