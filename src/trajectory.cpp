/*We use the Computed Torque libraries just to have the basic environment to work on */

//Mathematical
#include <iostream>
#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <std_msgs/String.h>

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/types.h>

//Franka
#include <franka/robot_state.h>
#include <franka_msgs/FrankaState.h>

//ROS message
#include <sensor_msgs/JointState.h>


/*Defining useful items*/

//ROS Time
double t_final;

//Variables
double dt; //parameter that is given from the topic
double t_i,t_f; //t_f is given from the topic
double dt_hat,t_star;
double toll; //tollerance

//Subscriber
ros::Subscriber sub_q_actual, sub_q_desired;
//Publisher
ros::Publisher pub_q_desired;

//Matrices
Eigen::Matrix<double,7,1> q_0;
Eigen::Matrix<double,7,1> q_final;
Eigen::Matrix<double,7,1> q_d_sym;
Eigen::Matrix<double,7,1> q_old;

//Flag for the new message
bool flag;

//Array
std::array<double,7> q_d_star;

//Message
sensor_msgs::JointState states;

//MultiSpinner
ros::MultiThreadedSpinner spinner(2);

void Position_Callback ( const franka_msgs::FrankaStatePtr& msg )   //point
{

     if ( ( msg->q ).size() != 7 || ( msg->q ).empty() ) {

          ROS_FATAL ( "Actual position has not dimension 7 or is empty!!!", ( msg->q ).size() );
     }
     //converting the array into a symbolic vector
     q_0 = Eigen::Map<Eigen::Matrix<double, 7, 1>> ( msg->q.data() );

     ROS_INFO ( "Actual pose of the joints:  ", q_0 );
}

void Desired_Callback ( const sensor_msgs::JointStatePtr& msg )
{

     if ( ( msg->position ).size() != 7 || ( msg->position ).empty() ) {

          ROS_FATAL ( "Desired position has not dimension 7 or is empty!!!", ( msg->position ).size() );
     }

     //convertion of the array into symbolic vector
     q_final = Eigen::Map<Eigen::Matrix<double, 7, 1>> ( msg->position.data() );

     ROS_INFO ( "Desired pose of the joints:  ", q_final );

     /*if the q_final changes the flag is set to false and t_final is resetted */

     flag = false;
     t_final = ros::Time::now().toSec(); //stopping the time
     q_old = q_final;

     std::cout<< "Status of the FLAG and t_final:    " << flag << "    " << t_final << std::endl;
}


int main ( int argc, char **argv )
{

     ros::init ( argc,argv, "trajectory" );
     ros::NodeHandle node_handle ( "~" );

     ROS_INFO ( "The NODE started working!\n" );

//    Sub_and_Pub object;

     //Finding the dt and t_f parameters
     if ( !node_handle.getParam ( "dt", dt ) || !node_handle.getParam ( "t_f",t_f ) ) { /*getting the parameter dt and t_f*/
          ROS_ERROR ( "Could not get parameter dt or t_f !" );
          return false;
     }

     ros::Rate frequency ( 1/dt ); //Declaring the frequency of sending the position

     t_i = 0; //setting initial time to zero
     toll = 0.0005; //need some test to choose the correct tollerance

     //Activating the subscribers
     sub_q_actual = node_handle.subscribe /*<franka_msgs::FrankaState>*/ ( "/franka_control/joint_states",1, Position_Callback );
     sub_q_desired = node_handle.subscribe /*<sensor_msgs::JointState>*/ ( "/desired_state",1, Desired_Callback );
     //Activating the publisher
     pub_q_desired = node_handle.advertise<sensor_msgs::JointState> ( "/panda_controllers/pd_controller/command",1 );

     std::cout<< "The tollerance chosen is:    " << toll << std::endl;

     q_0 = Eigen::MatrixXd::Identity ( 7,1 );

     std::cout << "The q_0 is:\n" << q_0 << std::endl;
     std::cout << "The  q_final is:\n" << q_final << std::endl;

     while ( ros::ok() ) {
       

          if ( flag == true ) {

               q_d_sym = q_0; //for entry in the loop

               while ( ( q_d_sym - q_final ).squaredNorm() > toll ) { /*End the cycle when we are in the desired joints position */

                    std::cout<< "We are in the cicle with FLAG TRUE." << std::endl;

                    t_star = t_star + dt_hat;
                    //Subdeviding the trajectory into small pieces dt_sign long
                    q_d_sym = q_0 + ( ( q_final - q_0 ) / ( t_f - t_i ) ) * ( t_star - t_i );

                    std::cout<< "Sampled trajectory:  " << q_d_sym.data() << std::endl;

                    //Convertion of the matrix into array
                    for ( int i=0; i<q_d_sym.rows(); i++ ) {
                         q_d_star[i] = q_d_sym[i];
                    }

                    std::cout<< "Array:  " << q_d_star.data() << std::endl;

                    //Trasfering the array in a string type
                    for ( int i=0; i<7; i++ ) {
                         states.position.push_back ( q_d_star[i] );
                    }

                    std::cout<< "Message:  " << states.position.data() << std::endl;

                    //publishing to command node the new q_desired
                    pub_q_desired.publish ( states.position );
		    
		    ros::spinOnce();

                    frequency.sleep();
                    
               }
               


          } else {

               dt_hat = 0;
               dt_hat = ros::Time::now().toSec() - t_final; //declaring the window for the sampling of the trajectory
               flag = true;

               ROS_INFO ( "We are in the else!!!" );
               std::cout<< "dt_hat:  " << dt_hat << std::endl;
               std::cout<< "Flag:  " << flag << std::endl;
          }
     }
     spinner.spin();

}








