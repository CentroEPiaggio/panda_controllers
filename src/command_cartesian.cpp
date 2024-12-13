#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

//#include <geometry_msgs/Point.h>
#include "panda_controllers/point.h"
#include "panda_controllers/desTrajEE.h"
#include "gen_traj_fun.h"

typedef Eigen::Vector3d vec3d;
typedef Eigen::Matrix<double,4,1> vec4d;

using std::cout;
using std::cin;
using std::endl;

/* End-effector current position */
vec3d pos_EE;
Eigen::Quaterniond or_EE;
vec3d pos_EE_start;
Eigen::Quaterniond or_EE_start;

/* Global flag */
bool init_start = false;
bool end_motion = false;
int flag_traj;

/* Define lissajous parameters */
double ampX, ampY, ampZ, freqX, freqY, freqZ, phiX, phiZ, offX, offY, offZ,liss_T;

/* Define minjerk parameters */
double minjerk_T;
double minjerk_xf, minjerk_yf, minjerk_zf;

/* Command duration */
double duration

/* variables for message */ ;
vec3d position_t, velocity_t, acceleration_t, ang_vel_t, ang_acc_t;
vec4d orientation_t;

/* Obtain end-effector pose */
void poseCallback(const panda_controllers::pointConstPtr& msg);

/* Trajectories */
void lissajous  (const double dt_, const vec3d p0);
void minjerk    (const double dt_, const vec3d p0);
void stay_in_p0 (const double dt_, const vec3d p0);
void trajFun    (const double dt_, const vec3d p0);
void computeOscillatingOrientation(double dt, const Eigen::Quaterniond& q0);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_node");
	ros::NodeHandle node_handle;
    double frequency = 1000;
	ros::Rate loop_rate(frequency);
	
	/* Publisher */
	ros::Publisher pub_cmd_cartesian = node_handle.advertise<panda_controllers::desTrajEE>("command_cartesian", 1000);
	
	/* Subscriber */
	ros::Subscriber sub_pose = node_handle.subscribe<panda_controllers::point>("current_config", 1, &poseCallback);

	/* Message for /backstepping_controller/command */
	panda_controllers::desTrajEE msg_cartesian;

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
        !node_handle.getParam("lissajous/duration", liss_T) ||
        !node_handle.getParam("lissajous/offZ", offZ) ||
        !node_handle.getParam("minjerk/xf", minjerk_xf) ||
        !node_handle.getParam("minjerk/yf", minjerk_yf) ||
        !node_handle.getParam("minjerk/zf", minjerk_zf) ||
        !node_handle.getParam("minjerk/duration", minjerk_T) ||
        !node_handle.getParam("flag/type", flag_traj)) {
		ROS_ERROR("Could not get trajectory parameters!");
		return false;
	}

    /* initialize trajectory pointer */
    void (*traj_ptr)(const double, const vec3d);
    
    switch (flag_traj)
    {
    case 1:
        traj_ptr = lissajous;
        duration = liss_T;
        break;
    case 2:
        traj_ptr = minjerk;
        duration = minjerk_T;
        break;
    case 3:
        traj_ptr = trajFun;
        duration = 50.0;
        break;
    case 4:
        traj_ptr = stay_in_p0;
        break;
    default:
        traj_ptr = stay_in_p0;
        break;
    }
    
    bool start = false;
	double t_start;
    double period = 1/frequency;
	ros::Time t;
    double dt = 0.0;

    //ros::Duration(2.0).sleep();

	while (ros::ok()){
    
        ros::spinOnce();
        t = ros::Time::now();

        if (!start){
            t_start = t.toSec();
            start = true;
        }else if(dt<(duration-period)){
            dt = t.toSec() - t_start;
        }else if(!end_motion){
            cout<<"\n=== tempo terminato ===\n";
            pos_EE_start = pos_EE;
			or_EE_start = or_EE;
            traj_ptr = stay_in_p0;
            end_motion = true;
        }
        
        //cout<<"\nend_motion: "<<end_motion<<"\npos_E_start: \n"<<pos_EE_start<<endl;
        (*traj_ptr)(dt, pos_EE_start);
        msg_cartesian.header.stamp = t;

        /* UPDATE MESSAGE */
        msg_cartesian.position.x = position_t(0);
        msg_cartesian.position.y = position_t(1);
        msg_cartesian.position.z = position_t(2);

        msg_cartesian.velocity.x = velocity_t(0);
        msg_cartesian.velocity.y = velocity_t(1);
        msg_cartesian.velocity.z = velocity_t(2);

        msg_cartesian.acceleration.x = acceleration_t(0);
        msg_cartesian.acceleration.y = acceleration_t(1);
        msg_cartesian.acceleration.z = acceleration_t(2);

		msg_cartesian.orientation.w = orientation_t(0);
        msg_cartesian.orientation.x = orientation_t(1);
        msg_cartesian.orientation.y = orientation_t(2);
		msg_cartesian.orientation.z = orientation_t(3);

		msg_cartesian.ang_vel.x = ang_vel_t(0);
        msg_cartesian.ang_vel.y = ang_vel_t(1);
        msg_cartesian.ang_vel.z = ang_vel_t(2);

		msg_cartesian.ang_acc.x = ang_acc_t(0);
        msg_cartesian.ang_acc.y = ang_acc_t(1);
        msg_cartesian.ang_acc.z = ang_acc_t(2);

        pub_cmd_cartesian.publish(msg_cartesian);  

		loop_rate.sleep();
	} 

	return 0;
}

void poseCallback(const panda_controllers::pointConstPtr& msg){

	double EE_x, EE_y, EE_z;
	
	EE_x = msg->xyz.x;
	EE_y = msg->xyz.y;
	EE_z = msg->xyz.z;

	pos_EE << EE_x, EE_y, EE_z;
	Eigen::Quaterniond or_EE_tmp(0.0268247, 0.922474, -0.381482, -0.0528501);
	or_EE = or_EE_tmp;
	// insert the pose callback with orientation
	// Eigen::Matrix3d rotation_matrix;
	// rotation_matrix << 	0.7031485305992036, -0.7067694330104566, -0.07784030111875523,
	// 					-0.7011826008400694, -0.7073983045971645, 0.08905391025784827,
	// 					-0.11800467870104367, -0.008037861353296398, -0.9929805076583974;
	// rotation_matrix = rotation_matrix.transpose();
	// or_EE = Eigen::Quaterniond(rotation_matrix);
	// or_EE.normalized();
	
	if (!init_start){
		// cout << "or_EE: " << or_EE.vec() << ", " << or_EE.w() << endl;
		pos_EE_start = pos_EE;
		// Eigen::Quaterniond or_EE_tmp(0.0268247, 0.922474, -0.381482, -0.0528501);
		or_EE_start = or_EE;
		init_start = true;
        std::cout<<"\n==============\n"<<"pos_EE_start:"<<"\n==============\n"<<pos_EE_start<<"\n==============\n";
	}
}

void lissajous(const double dt_, const vec3d p0){
        
    double x0,y0,z0,dt;

    double A = ampX, B = ampY, C = ampZ;
    double a = freqX, b = freqY, c = freqZ;
    double dx = phiX, dz = phiZ;
    double t0 = 1.0/(4*b);
    //std::cout<<t0<<std::endl;
    dt = dt_;
/*     x0 = p0(0) + offX;
    y0 = p0(1) + offY;
    z0 = p0(2) + offZ; */
    x0 = 0.380 + offX;
    y0 = 0.000 + offY;
    z0 = 0.400 + offZ;

    position_t << 
        x0 + A * std::sin(2*M_PI * a * (dt-t0) + dx),
        y0 + B * std::cos(2*M_PI * b * (dt-t0)),
        z0 + C * std::sin(2*M_PI * c * (dt-t0) + dz);
    velocity_t << 
        2*M_PI *A * a * std::cos(2*M_PI * a * (dt-t0) + dx),
        -2*M_PI *B * b * std::sin(2*M_PI * b * (dt-t0)),
        2*M_PI *C * c * std::cos(2*M_PI * c * (dt-t0) + dz);
    acceleration_t << 
        -2*M_PI *2*M_PI *A * a * a * std::sin(2*M_PI * a * (dt-t0) + dx),
        -2*M_PI *2*M_PI *B * b * b * std::cos(2*M_PI * b * (dt-t0)), 
        -2*M_PI *2*M_PI *C * c * c * std::sin(2*M_PI * c * (dt-t0) + dz);

	// --- Orientation --- //
	
	// orientation_t << 
    //     x0 + A * std::sin(2*M_PI * a * (dt-t0) + dx),
    //     y0 + B * std::cos(2*M_PI * b * (dt-t0)),
    //     z0 + C * std::sin(2*M_PI * c * (dt-t0) + dz);
	// Eigen::Quaterniond q0(1.0, 0.0, 0.0, 0.0);
	// q0 << 1.0, 0.0, 0.0, 0.0;
	// computeOscillatingOrientation(dt, or_EE_start);
}

// Eigen::Quaterniond quatMul(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
// 	Eigen::Quaterniond resultQ;

// 	resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
// 	resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

// 	resultQ.normalized();

// 	return resultQ;
// }

void computeOscillatingOrientation(double dt, const Eigen::Quaterniond& q0){
	double t0 = 0;
	double omega = 1;
	double A = 0.1;
	double phase = 0;
	Eigen::Vector3d rotation_axis;
	rotation_axis << 0, 0, 1;

	Eigen::Vector3d axis = rotation_axis.normalized();

	// axis: θ(t) = A * sin(ωt + φ)
	double theta = omega*dt; //A * std::sin(omega * (dt-t0) + phase);

	// angular velocity: θ'(t) = A * ω * cos(ωt + φ)
	double theta_dot = 1; //A * omega * std::cos(omega * (dt-t0) + phase);

	// angular acceleration: θ''(t) = -A * ω^2 * sin(ωt + φ)
	double theta_ddot = 0; // - A * omega * omega * std::sin(omega * (dt-t0) + phase);

	// quaternion from agle-axis
	Eigen::Quaterniond q_t(Eigen::AngleAxisd(theta, axis));
	// Eigen::Quaterniond q_t(
	// 	std::cos(theta / 2),
	// 	std::sin(theta / 2) * axis.x(),
	// 	std::sin(theta / 2) * axis.y(),
	// 	std::sin(theta / 2) * axis.z()
	// );

	// orientation
	// Eigen::Quaterniond orientation = quatMul(q0, q_t);
	Eigen::Quaterniond orientation = q_t * q0;

	// Velocità angolare nel frame globale: ω = θ'(t) * axis
	ang_vel_t = theta_dot * axis;

	// Accelerazione angolare nel frame globale: α = θ''(t) * axis
	ang_acc_t = theta_ddot * axis;

	// Salva l'orientazione attuale
	orientation_t << orientation.w(), orientation.x(), orientation.y(), orientation.z();
}

void minjerk(const double dt_, const vec3d p0){
        
    vec3d start, end;

    start << p0(0), p0(1), p0(2);
    end << minjerk_xf, minjerk_yf, minjerk_zf;
    
    position_t << start + (start - end)*(15*pow((dt_/duration),4) - 6*pow((dt_/duration),5) -10*pow((dt_/duration),3));
	velocity_t << (start - end)*(60*(pow(dt_,3)/pow(duration,4)) - 30*(pow(dt_,4)/pow(duration,5)) -30*(pow(dt_,2)/pow(duration,3)));
	acceleration_t << (start - end)*(180*(pow(dt_,2)/pow(duration,4)) - 120*(pow(dt_,3)/pow(duration,5)) -60*(dt_/pow(duration,3)));
}

void stay_in_p0(const double dt_, const vec3d p0){

    position_t<< p0(0), p0(1), p0(2);
    velocity_t.setZero();
    acceleration_t.setZero();

	Eigen::Quaterniond or_EE_tmp(0.0268247, 0.922474, -0.381482, -0.0528501);
	Eigen::Quaterniond orientation = or_EE_tmp;
	orientation_t << orientation.w(), orientation.x(), orientation.y(), orientation.z();
	ang_vel_t.setZero();
	ang_acc_t.setZero();
}

void trajFun(const double dt_, const vec3d p0){
    
    long long p3_p[traj_pos_SZ_IW];
    double p4_p[traj_pos_SZ_W];
    long long p3_v[traj_vel_SZ_IW];
    double p4_v[traj_vel_SZ_W];
    long long p3_a[traj_acc_SZ_IW];
    double p4_a[traj_acc_SZ_W];

    const double* input_[] = {&dt_};
    double* output_pos[] = {position_t.data()};
    double* output_vel[] = {velocity_t.data()};
    double* output_acc[] = {acceleration_t.data()};
    int check;

    check = traj_pos(input_, output_pos, p3_p, p4_p, 0);
    check = traj_vel(input_, output_vel, p3_v, p4_v, 0);
    check = traj_acc(input_, output_acc, p3_a, p4_a, 0);

    position_t[0] = position_t[0]+p0[0];
    position_t[1] = position_t[1]+p0[1];
    position_t[2] = position_t[2]+p0[2];
}