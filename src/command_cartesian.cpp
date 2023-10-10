#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

//#include <geometry_msgs/Point.h>
#include "panda_controllers/point.h"
#include "panda_controllers/desTrajEE.h"
#include "utils/gen_traj_fun.h"

typedef Eigen::Vector3d vec3d;

using std::cout;
using std::cin;
using std::endl;

/* End-effector current position */
vec3d pose_EE;
vec3d pose_EE_start;

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
vec3d position_t, velocity_t, acceleration_t;

/* Obtain end-effector pose */
void poseCallback(const panda_controllers::pointConstPtr& msg);

/* Trajectories */
void lissajous  (const double dt_, const vec3d p0);
void minjerk    (const double dt_, const vec3d p0);
void stay_in_p0 (const double dt_, const vec3d p0);
void trajFun    (const double dt_, const vec3d p0);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_node");
	ros::NodeHandle node_handle;
    double frequency = 100;
	ros::Rate loop_rate(frequency); // 100 Hz,10 volte più lento del controllore
	
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

    ros::Duration(1.0).sleep();

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
            pose_EE_start = pose_EE;
            traj_ptr = stay_in_p0;
            end_motion = true;
        }
        
        //cout<<"\nend_motion: "<<end_motion<<"\npos_E_start: \n"<<pose_EE_start<<endl;
        (*traj_ptr)(dt,pose_EE_start);
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

	pose_EE << EE_x, EE_y, EE_z;

	if (!init_start){
		pose_EE_start = pose_EE;
		init_start = true;
        std::cout<<"\n==============\n"<<"pose_EE_start:"<<"\n==============\n"<<pose_EE_start<<"\n==============\n";
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
    x0 = p0(0) + offX;
    y0 = p0(1) + offY;
    z0 = p0(2) + offZ;

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