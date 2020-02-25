#include <panda_controllers/command_publisher.h>

using namespace panda_controllers;
/* Se non dichiari il namespace usato dalla classe, allora il costruttore lo crei in questo modo:
 * panda_controllers::Command_Publisher Pippo(&nh);
 * 
 */
int main (int argc, char **argv)
{
    ros::init(argc, argv, "command_publisher");
    ros::NodeHandle nh("/command_publisher");
    
    /* Create an istance of Command_Publisher class */
    
    Command_Publisher publish(nh);
    
    ROS_INFO("Node started");
    
    std::cout << "t_f parameter is " << publish.get_t_f() << std::endl;
    std::cout << "dt parameter is " << publish.get_d_t() << std::endl;
    
    while(publish.get_index() == false){
      ros::spinOnce();
      ROS_DEBUG_STREAM("Waiting for a desired command position!"); 
    }   
    publish.publish_command();

    return 0;    
}