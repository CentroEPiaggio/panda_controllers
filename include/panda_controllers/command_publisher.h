#include <ros/ros.h>
#include <ros/time.h>
#include <ros/rate.h>
#include <Eigen/Dense>
#include <franka_msgs/FrankaState.h> /*to get the current robot joint state*/
#include <sensor_msgs/JointState.h>

#ifndef COMMMAND_PUBLISHER  
#define COMMAND_PUBLISHER  

namespace panda_controllers
{

class Command_Publisher
{
private:
  
  bool index = false; 	//flag to check the command position on the command_q_f_callback
  double dt;     	//dt time interval
  double frequency;
 
  /* Ros time variables: t f-> t final; t_0 -> t initial; t_last, dt_bar, and elapsed_time to calculate the spent time */
  std::shared_ptr<ros::Rate> rate;
  ros::Time t_0;
  double t_f;  
  ros::Time t_last;
  ros::Duration dt_bar; 
  ros::Duration elapsed_time;
    
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> dot_q;
  Eigen::Matrix<double, 7, 1> q_initial;
  Eigen::Matrix<double, 7, 1> q_final;
  Eigen::Matrix<double, 7, 1> q_des_command;
  std::array<double,7> prova;
  sensor_msgs::JointState q_des_command_array;
  
  /* Ros publisher and subscriber */
  
  ros::Subscriber sub1;
  ros::Subscriber sub2;
  ros::Publisher pub;
  
  ros::NodeHandle nh_;
  
public:
  
  /* Define costructor*/
  
  Command_Publisher(ros::NodeHandle& nh):nh_(nh){
    
    // Get name of controller
    std::string controller;
    if(nh_.getParam("controller", controller)){
    ROS_INFO("Get controller name.");
    } else {
      ROS_ERROR("Could not get controller name!");
    }
    
    this->pub = nh_.advertise<sensor_msgs::JointState>("/panda_controllers/" + controller + "/command", 1);
    this->sub1 = nh_.subscribe("/Franka_Joint_State", 1, &Command_Publisher::franka_callback, this);
    this->sub2 = nh_.subscribe("command_q_f", 1, &Command_Publisher::command_q_f_callback, this); 
    
    if(nh_.getParam("dt", this->dt) && nh_.getParam("t_f", this->t_f)){
      ROS_INFO("Get parameters");
    } else {
      ROS_ERROR("Could not get dt or t_f parameter!");
    }
    
    this->frequency = 1/this->dt;
    this->rate = std::make_shared<ros::Rate>(this->frequency);
    
    std::cout << "controller parameter is " << controller << std::endl;
    std::cout << "t_f parameter is " << this->t_f << std::endl;
    std::cout << "dt parameter is " << this->dt << std::endl;
  }
  
  /* Define Destructor */ 
  
  ~Command_Publisher() = default;
  
  bool get_index(){return index;} 
  
  double get_frequency(){return frequency;}
  
  double get_t_f(){return t_f;};
  
  double get_d_t(){return dt;};
   
  void franka_callback(const franka_msgs::FrankaStatePtr& joint_state){
    
    q = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((joint_state->q).data());
    dot_q = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((joint_state->dq).data());  
  }
  
  void command_q_f_callback(const sensor_msgs::JointStatePtr& msg){
    
    if((msg->position).size() == 7){  
    
    q_initial = q;
    t_0 = ros::Time(0.0);
    elapsed_time = ros::Duration(0.0);
    t_last = ros::Time::now();
    
    /* Saving the last desired command position */
    
    q_final = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
    index = true;    
    }
  }  
  
  void publish_command(){
    
    t_last = ros::Time::now();
  
    /* Necessary for allocate memory before assignment, because q_des_command_array has an undefined dimension */
    /* Assing the dimension BEFORE while(ros::ok()) loop */
    for(int i = 0; i < 7; ++i){
      q_des_command_array.position.push_back(0.0);
    }

    while(ros::ok()){ 
    
    std::cout<< "Checkpoint q_initial " << q_initial << std::endl;
    
    ros::spinOnce();    
    dt_bar = ros::Time::now() - t_last;
    this->t_last = ros::Time::now();
    elapsed_time += dt_bar; 
    
    std::cout << "Checkpoint 2" << std::endl; 
    
    q_des_command = q_initial + (q_final - q_initial) / (t_f - t_0.toSec()) * (elapsed_time.toSec() - t_0.toSec()); 
    
    std::cout << " Checkpoint q_des_command :" << q_des_command << std::endl;
    std::cout << "Checkpoint 3 " << std::endl;
       
//     std::cout << q_des_command[0] << "\n";
//     std::cout << q_des_command[1] << "\n";
//     std::cout << q_des_command[2] << "\n";
//     std::cout << q_des_command[3] << "\n";
//     std::cout << q_des_command[4] << "\n";
//     std::cout << q_des_command[5] << "\n";
//     std::cout << q_des_command[6] << "\n";
      
//     /* Necessary for allocate memory before assignment, because q_des_command_array has an undefined dimension */
//     
//     for(int i = 0; i < 7; ++i){
//       q_des_command_array.position.push_back(0.0);
//     }
     std::cout << "Dimension of q_des_command_array before assignment " << q_des_command_array.position.size() << "\n";
//     std::cout << q_des_command_array.position[0] << "\n";
//     std::cout << q_des_command_array.position[1] << "\n";
//     std::cout << q_des_command_array.position[2] << "\n";
//     std::cout << q_des_command_array.position[3] << "\n";
//     std::cout << q_des_command_array.position[4] << "\n";
//     std::cout << q_des_command_array.position[5] << "\n";
//     std::cout << q_des_command_array.position[6] << "\n";
    
    for(int i = 0; i < 7; ++i){
      q_des_command_array.position[i] = q_des_command[i];
      std::cout << q_des_command_array.position[i] << "\n";
    }

    std::cout << " Checkpoint 4 " << std::endl;
    
    /* publish message */
    std::cout << "Dimension of q_des_command_array after assignment " << q_des_command_array.position.size() << "\n";
    
    pub.publish(q_des_command_array);   
    
    std::cout << "Dimensione of q_des_command_array after publish " << q_des_command_array.position.size() << "\n";
    
    rate->sleep(); 
    std::cout << "Checkpoint 5 " << std::endl;
    }
  }   
};
}

#endif