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
  double tol = 1e-6;
 
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
  Eigen::Matrix<double, 7, 1> q_d_command;
  
  sensor_msgs::JointState q_d_command_array;
  
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
      
      q_initial = q; /* Save the current joints position*/
//    q_initial << 0, 3, 4, 5, 2, 1, 3; 
      t_0 = ros::Time(0.0);
      elapsed_time = ros::Duration(0.0);
      t_last = ros::Time::now();
    
      /* Saving the last desired command position */
    
      q_final = Eigen::Map<const Eigen::Matrix<double, 7, 1>>((msg->position).data());
      index = true;    
    } else {
      
      ROS_ERROR("Wrong position dimension !!!!!!");         
    }
  }
  
  void publish_command(){
       
    t_last = ros::Time::now();
  
    /* Necessary for allocate memory before assignment, because q_des_command_array has an undefined dimension */

    q_d_command_array.position.resize(7,1);
    q_d_command_array.velocity.resize(7,1);
    
    index = false; /* Reset flag */  
    
    do{ 
      
      //std::cout<< "Checkpoint q_initial " << q_initial << std::endl;
      ros::spinOnce();    
      dt_bar = ros::Time::now() - t_last;
      this->t_last = ros::Time::now();
      elapsed_time += dt_bar; 
        
      q_d_command = q_initial + (q_final - q_initial) / (t_f - t_0.toSec()) * (elapsed_time.toSec() - t_0.toSec());  

      for(int i = 0; i < 7; ++i){
	
      q_d_command_array.position[i] = q_d_command[i];
      
      }
    
      /* publish message */
     
      for(int i =0; i < 7; ++i){
      std::cout << "q_d_command_array " << q_d_command_array.position[i] << "\n";
      }
       
      pub.publish(q_d_command_array);     
      rate->sleep(); 
      std::cout << "Last checkpoint "<< std::endl; 
      
      }while((q_d_command - q_final).squaredNorm() >= tol);    
  }// close void function
};
}

#endif

    
