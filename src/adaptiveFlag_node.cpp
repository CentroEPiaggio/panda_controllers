#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include "panda_controllers/flag.h"


typedef Eigen::Vector3d vec3d;

using std::cout;
using std::cin;
using std::endl;

bool update_flag_ = true;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "adaptiveFlag_node");
	ros::NodeHandle node_handle;
    double frequency = 100;
	ros::Rate loop_rate(frequency); // 100 Hz,10 volte più lento del controllore
	
	/* Publisher */
	ros::Publisher pub_flagAdaptive = node_handle.advertise<panda_controllers::flag>("adaptiveFlag", 1);

	panda_controllers::flag msg_update_flag;

	ros::Time t;
    
	while (ros::ok()){
    
        ros::spinOnce();

        t = ros::Time::now();

        /* to do: some law for setting adaptive flag */

        msg_update_flag.flag = update_flag_;
        msg_update_flag.header.stamp = t;

        pub_flagAdaptive.publish(msg_update_flag);  
		loop_rate.sleep();
	} 

	return 0;
}

