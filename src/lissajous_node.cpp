#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

//#include <geometry_msgs/Point.h>
#include "panda_controllers/point.h"
#include "panda_controllers/desTrajEE.h"

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

/* Define line parameters */
double line_T;
double xf, yf, zf;

/* Define minjerk parameters */
double minjerk_T;
double minjerk_xf, minjerk_yf, minjerk_zf;

/* command duration */
double duration;

/* Obatain end-effector pose */
void poseCallback(const panda_controllers::point& msg);

/* prototipi traiettorie */
void lissajous  (const double dt_, const vec3d p0, panda_controllers::desTrajEE &msg);
void line       (const double dt_, const vec3d p0, panda_controllers::desTrajEE &msg);
void minjerk    (const double dt_, const vec3d p0, panda_controllers::desTrajEE &msg);
void stay_in_p0 (const double dt_, const vec3d p0, panda_controllers::desTrajEE &msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lissajous");
	ros::NodeHandle node_handle;
    double frequency = 100;
	ros::Rate loop_rate(frequency); // 100 Hz,10 volte più lento del controllore
	
	/* Publisher */
	ros::Publisher pub_cmd_cartesian = node_handle.advertise<panda_controllers::desTrajEE>("/backstepping_controller/command", 1);
	
	/* Subscriber */
	ros::Subscriber sub_pose = node_handle.subscribe("/backstepping_controller/current_config", 1, &poseCallback);

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
        !node_handle.getParam("line/xf", xf) ||
        !node_handle.getParam("line/yf", yf) ||
        !node_handle.getParam("line/zf", zf) ||
        !node_handle.getParam("line/duration", line_T) ||
        !node_handle.getParam("lissajous/offZ", offZ) ||
        !node_handle.getParam("minjerk/xf", minjerk_xf) ||
        !node_handle.getParam("minjerk/yf", minjerk_yf) ||
        !node_handle.getParam("minjerk/zf", minjerk_zf) ||
        !node_handle.getParam("minjerk/duration", minjerk_T) ||
        !node_handle.getParam("flag/type", flag_traj)) {
		ROS_ERROR("Could not get lissajous parameters!");
		return false;
	}

    /* initialize trajectory pointer */
    void (*traj_ptr)(const double, const vec3d, panda_controllers::desTrajEE&);
    
    switch (flag_traj)
    {
    case 1:
        traj_ptr = lissajous;
        duration = liss_T;
        break;
    case 2:
        traj_ptr = line;
        duration = line_T;
        break;
    case 3:
        traj_ptr = minjerk;
        duration = minjerk_T;
        break;
    default:
        traj_ptr = stay_in_p0;
        break;
    }
    

    bool start = false;
	double t_start;
    double period = 1/frequency;
/*     cout<<"period: "<<period<<endl; */
	ros::Time t;
    double dt = 0;

    ros::Duration(0.1).sleep();

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
/* 
        cout<<"\nend_motion: "<<end_motion<<"\npos_E_start: \n"<<pose_EE_start<<endl; */
        (*traj_ptr)(dt,pose_EE_start,msg_cartesian);
        msg_cartesian.header.stamp = t;

        pub_cmd_cartesian.publish(msg_cartesian);  

		loop_rate.sleep();
	} 

	return 0;
}

void poseCallback(const panda_controllers::point& msg){

	double EE_x, EE_y, EE_z;
	
	EE_x = msg.xyz.x;
	EE_y = msg.xyz.y;
	EE_z = msg.xyz.z;

	pose_EE << EE_x, EE_y, EE_z;

	if (!init_start){
		pose_EE_start = pose_EE;
		init_start = true;
	}
}

void lissajous(const double dt_, const vec3d p0, panda_controllers::desTrajEE &msg){
        
    double x0,y0,z0;
    vec3d position_t, velocity_t, acceleration_t;

    double A = ampX, B = ampY, C = ampZ;
    double a = freqX, b = freqY, c = freqZ;
    double dx = phiX, dz = phiZ;
    double t0 = 1/(2*b)*0;

    x0 = p0(0) + offX;
    y0 = p0(1) + offY;
    z0 = p0(2) + offZ;

    position_t << 
        x0 + A * std::sin(M_2_PI * a * (dt_-t0) + dx),
        y0 + B * std::cos(M_2_PI * b * (dt_-t0)),
        z0 + C * std::sin(M_2_PI * c * (dt_-t0) + dz);
    velocity_t << 
        M_2_PI *A * a * std::cos(M_2_PI * a * (dt_-t0) + dx),
        -M_2_PI *B * b * std::sin(M_2_PI * b * (dt_-t0)),
        M_2_PI *C * c * std::cos(M_2_PI * c * (dt_-t0) + dz);
    acceleration_t << 
        -M_2_PI *M_2_PI *A * a * a * std::sin(M_2_PI * a * (dt_-t0) + dx),
        -M_2_PI *M_2_PI *B * b * b * std::cos(M_2_PI * b * (dt_-t0)), 
        -M_2_PI *M_2_PI *C * c * c * std::sin(M_2_PI * c * (dt_-t0) + dz);


    /* UPDATE MESSAGE */
    msg.position.x = position_t(0);
    msg.position.y = position_t(1);
    msg.position.z = position_t(2);

    msg.velocity.x = velocity_t(0);
    msg.velocity.y = velocity_t(1);
    msg.velocity.z = velocity_t(2);

    msg.acceleration.x = acceleration_t(0);
    msg.acceleration.y = acceleration_t(1);
    msg.acceleration.z = acceleration_t(2);
}

void line(const double dt_, const vec3d p0, panda_controllers::desTrajEE &msg){
        
    vec3d start, end, lambda;
    double d, Delta_t;
    double pos, vel, acc, A;
    vec3d position_t, velocity_t, acceleration_t;

    start << p0(0), p0(1), p0(2);
    end << xf, yf, zf;
    Delta_t = duration;

    lambda = end-start; 
    d = lambda.norm(); // distance between points 
    lambda = lambda/d; // direction of motion
    vel = d/Delta_t;   // velocity of motion

    position_t = start + vel* dt_ * lambda;
    velocity_t = vel * lambda;
    acceleration_t.Zero();

    /* UPDATE MESSAGE */
    msg.position.x = position_t(0);
    msg.position.y = position_t(1);
    msg.position.z = position_t(2);

    msg.velocity.x = velocity_t(0);
    msg.velocity.y = velocity_t(1);
    msg.velocity.z = velocity_t(2);

    msg.acceleration.x = acceleration_t(0);
    msg.acceleration.y = acceleration_t(1);
    msg.acceleration.z = acceleration_t(2);
}

void minjerk(const double dt_, const vec3d p0, panda_controllers::desTrajEE &msg){
        
    vec3d start, end;
    double Delta_t;
    vec3d position_t, velocity_t, acceleration_t;

    start << p0(0), p0(1), p0(2);
    end << minjerk_xf, minjerk_yf, minjerk_zf;
    Delta_t = duration;
    
    position_t << start + (start - end)*(15*pow((dt_/duration),4) - 6*pow((dt_/duration),5) -10*pow((dt_/duration),3));
	velocity_t << (start - end)*(60*(pow(dt_,3)/pow(duration,4)) - 30*(pow(dt_,4)/pow(duration,5)) -30*(pow(dt_,2)/pow(duration,3)));
	acceleration_t << (start - end)*(180*(pow(dt_,2)/pow(duration,4)) - 120*(pow(dt_,3)/pow(duration,5)) -60*(dt_/pow(duration,3)));

    /* UPDATE MESSAGE */
    msg.position.x = position_t(0);
    msg.position.y = position_t(1);
    msg.position.z = position_t(2);

    msg.velocity.x = velocity_t(0);
    msg.velocity.y = velocity_t(1);
    msg.velocity.z = velocity_t(2);

    msg.acceleration.x = acceleration_t(0);
    msg.acceleration.y = acceleration_t(1);
    msg.acceleration.z = acceleration_t(2);
}

void stay_in_p0(const double dt_, const vec3d p0, panda_controllers::desTrajEE &msg){
        
    vec3d position_t, velocity_t, acceleration_t;

    position_t<< p0(0), p0(1), p0(2);
    std::cout<<position_t<<std::endl;
    velocity_t.Zero();
    acceleration_t.Zero();

    /* UPDATE MESSAGE */
    msg.position.x = position_t(0);
    msg.position.y = position_t(1);
    msg.position.z = position_t(2);

    msg.velocity.x = velocity_t(0);
    msg.velocity.y = velocity_t(1);
    msg.velocity.z = velocity_t(2);

    msg.acceleration.x = acceleration_t(0);
    msg.acceleration.y = acceleration_t(1);
    msg.acceleration.z = acceleration_t(2);
}
