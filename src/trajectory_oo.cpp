
#include <panda_controllers/trajectory_oo.h>

using namespace panda_controllers ;

sub_pub::sub_pub ( ros::NodeHandle* node_handle )
{
     ROS_INFO ( "Inizialization of the CLASS" );

     //inizialization of all the functions
     initializeSub_actual();
     initializeSub_desired();
     initializePub();

     //At the starting of the node, we set to zero the variables.
     q_0 = Eigen::MatrixXd::Identity ( 7,1 );
     q_d_sym = q_0;
     q_final = q_0;
     q_current = q_0;

}

void sub_pub::initializeSub_actual() //subscriber of the joints coming from Franka
{

     ROS_INFO ( "Inizialization of the subscriber for ACTUAL JOINTS POSE\n" );
     sub_q_actual = node_handle.subscribe ( "/franka_control/joint_states", 1, &sub_pub::Position_Callback, this );
}

void  sub_pub::initializeSub_desired() //subscriber of the desired pose coming from the user
{
     ROS_INFO ( "Inizialization of the subscriber for DESIRED JOINTS POSE\n" );
     sub_q_desired = node_handle.subscribe ( "desired_state", 1, &sub_pub::Desired_Callback, this );
}

void  sub_pub::initializePub() //publisher of the sampled trajectory
{
     ROS_INFO ( "Inizialization of the publisher\n" );
     pub_q_desired = node_handle.advertise<sensor_msgs::JointState> ( "/panda_controllers/command", 1 );
}

/* The two callbacks of the subscribers  */
void  sub_pub::Position_Callback ( const franka_msgs::FrankaStatePtr& msg )
{
     if ( ( msg->q ).size() != 7 || ( msg->q ).empty() ) {
          ROS_FATAL ( "Actual position has not dimension 7 or is empty!!!", ( msg->q ).size() );
     }

     //converting the array into a symbolic vector
     q_current = Eigen::Map<const Eigen::Matrix<double, 7, 1>> ( msg->q.data() );
     ROS_INFO ( "Actual pose of the joints:  ", q_0.data() );
}

void  sub_pub::Desired_Callback ( const sensor_msgs::JointStatePtr& msg )
{
     if ( ( msg->position ).size() != 7 || ( msg->position ).empty() ) {
          ROS_FATAL ( "Desired position has not dimension 7 or is empty!!!", ( msg->position ).size() );
     }

     //convertion of the array into symbolic vector
     q_final = Eigen::Map<const Eigen::Matrix<double, 7, 1>> ( msg->position.data() );

     ROS_INFO ( "Desired position of the joints:  ", q_final.data() );

     /*if the q_final changes the flag is set to false */
     flag = false;
}

void sub_pub::sampling_trajectory ( double dt , double t_f ) //The trajectory sampled
{
     //Declaring the frequency of messaging
     frequency = 1/dt;
     rate = std::make_shared<ros::Rate> ( frequency );

     std::cout<< "Flag Check!" << std::endl;

     if ( flag ) {

          ros::spinOnce();

          std::cout<< "We are in the cicle with FLAG TRUE." << std::endl;

          while ( ( q_d_sym - q_final ).squaredNorm() > toll && flag ) {

               ros::spinOnce();

               dt_hat = ros::Time::now().toSec() - t_final; //Updating the dt_hat, we want to gather smaller windows.
               t_star = t_star + dt_hat;
               t_final = ros::Time::now().toSec();

               //Subdeviding the trajectory into small pieces dt_sign long
               q_d_sym = q_0 + ( ( q_final - q_0 ) / ( t_f - t_i ) ) * ( t_star - t_i );

               std::cout << "q_d_sim: " << q_d_sym << std::endl;

               std::cout<< "Sampled trajectory:  \n" << q_d_sym.data() << std::endl;

               //Convertion of the MATRIX INTO ARRAY
               for ( int i=0; i<7; i++ ) {
                    q_d_star[i] = q_d_sym[i];
               }

               std::cout<< "Array:  \n" << q_d_star.data() << std::endl;

               //Trasfering the ARRAY INTO STRING type
               for ( int i=0; i<7; i++ ) {
                    states.position.push_back ( q_d_star[i] );
                    std::cout << q_d_star[i] << std::endl;
               }
               std::cout << " " << std::endl;

               std::cout<< "Message:  \n" << states.position.data() << std::endl;

               //publishing to command node the new q_desired
               pub_q_desired.publish ( states );

               std::cout << "Publish:   " << pub_q_desired << std::endl;


               rate->sleep();
          }

     } else { //We enter in the else when a new q_final comes. Is the "resetting" cycle.

          t_star = 0.0;
          dt_hat = 0;
          t_final = ros::Time::now().toSec();
          flag = true;
          std::cout<< "EXITING from the ELSE." << std::endl;
     }

}



int main ( int argc, char **argv )
{
     ros::init ( argc,argv, "trajectory_oo" );
     ros::NodeHandle nh ( "~" );

     //Defining useful variables
     double ti;
     double toll;
     double t_f, dt;

     //Finding the dt and t_f parameters
     if ( !nh.getParam ( "/trajectory_oo/dt", dt ) || !nh.getParam ( "/trajectory_oo/t_f", t_f ) ) {
          ROS_ERROR ( "Could not get parameter dt or t_f !" );
          return 1;
     }

     std::cout << "Value of dt and t_f:    " << dt << "    " << t_f << std::endl;

     //Defining initial TIME and the TOLLERANCE
     ti = 0;
     toll = 0.0005;

     ROS_INFO ( "Node active, ready to work!" );
     sub_pub sub_pub_inst ( &nh );

     while ( ros::ok() ) {
          sub_pub_inst.sampling_trajectory ( dt,t_f );
     }

     return 0;

}


