#include <panda_controllers/command_publisher.h>

using namespace panda_controllers;
/* Se non dichiari il namespace usato dalla classe, allora il costruttore lo crei in questo modo:
 * panda_controllers::Command_Publisher Pippo(&nh);
 * 
 *
 */
int main (int argc, char **argv)
{
    ros::init(argc, argv, "command_publisher");
    ros::NodeHandle nh("/command_publisher");
    
    /* Create an istance of Command_Publisher class */
    
    Command_Publisher publisher(nh);
    
    ROS_INFO("Node started");
    
    std::cout << "t_f parameter is " << publisher.get_t_f() << std::endl;
    
    std::cout << "dt parameter is " << publisher.get_d_t() << std::endl;
    
    while(publisher.get_frequency() == false){
      ros::spinOnce();
      ROS_DEBUG_STREAM("Waiting for a desired command position!"); 
    }   
    publisher.publish_command();
    
    //TO DO: Missing ros::SpinOnce()?? Lo devo mettere qua, oppure nella callback?    
    return 0;    
}

/*con il . accedi ai metodi dell'istanza della classe....
  Per esempio:
  publisher. ---> seleziona tutti i metodi(membri pubblici della classe Command_Publisher) che ti permette di accedere
  ai membri privati
 */


