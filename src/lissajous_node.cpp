#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <geometry_msgs/Point.h>
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

/* Define lissajous parameters */
double ampX, ampY, ampZ, freqX, freqY, freqZ, phiX, phiZ;
  
/* Obatain end-effector pose */
void poseCallback(const geometry_msgs::Point& msg);

/* prototipi traiettorie */
void lissajous(const double dt_, const vec3d p0, panda_controllers::desTrajEE &msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lissajous");
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(100); // 100 Hz,10 volte più lento del controllore
	ros::Time t_init;
	
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
        !node_handle.getParam("lissajous/phiZ", phiZ)) {
		ROS_ERROR("Could not get lissajous parameters!");
		return false;
	}
    
    pose_EE_start << 0.5,0,0.5;

    /* initialize trajectory pointer */
    void (*traj_ptr)(const double, const vec3d, panda_controllers::desTrajEE&);
    
    traj_ptr = lissajous;

    bool start = false;
	ros::Time t_start;
	ros::Time t;
    double dt = 0;
	//double duration = 20; // sec

	while (ros::ok()){
 
		ros::spinOnce();
        t = ros::Time::now();

        if (!start){
            t_start = t;
            start = true;
        }

        dt = t.toSec() - t_start.toSec();

        (*traj_ptr)(dt,pose_EE_start,msg_cartesian);

        msg_cartesian.header.stamp = t;

		pub_cmd_cartesian.publish(msg_cartesian);

		loop_rate.sleep();
	}

	return 0;
}

void poseCallback(const geometry_msgs::Point& msg){

	double EE_x, EE_y, EE_z;
	
	EE_x = msg.x;
	EE_y = msg.y;
	EE_z = msg.z;

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
    double t0 = 1/(4*b);

    x0 = p0(0);
    y0 = p0(1);
    z0 = p0(2);

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

