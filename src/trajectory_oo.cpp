
#include <panda_controllers/trajectory_oo.h>

using namespace panda_controllers ;

sub_pub::sub_pub ( ros::NodeHandle* node_handle )
{
     ROS_INFO ( "Inizialization of the CLASS" );
     initializeSub_actual();
     initializeSub_desired();
     initializePub();
     sampling_trajectory(this->dt, this->t_f);

}

void sub_pub::initializeSub_actual()
{

     ROS_INFO ( "Inizialization of the subscriber for ACTUAL JOINTS POSE\n" );
     sub_q_actual = node_handle.subscribe ( "/franka_control/joint_states", 1, &sub_pub::Position_Callback, this );
}

void  sub_pub::initializeSub_desired()
{
     ROS_INFO ( "Inizialization of the subscriber for DESIRED JOINTS POSE\n" );
     sub_q_desired = node_handle.subscribe ( "desired_state", 1, &sub_pub::Desired_Callback, this );

     t_final = ros::Time::now().toSec(); //stopping the time
}

void  sub_pub::initializePub()
{
     ROS_INFO ( "Inizialization of the publisher\n" );
     pub_q_desired = node_handle.advertise<sensor_msgs::JointState> ( "/panda_controllers/pd_controller/command", 1 );
}


void  sub_pub::Position_Callback ( const franka_msgs::FrankaStatePtr& msg )
{
     if ( ( msg->q ).size() != 7 || ( msg->q ).empty() ) {
          ROS_FATAL ( "Actual position has not dimension 7 or is empty!!!", ( msg->q ).size() );
     }

     //converting the array into a symbolic vector
     q_0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>> ( msg->q.data() );
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

     /*if the q_final changes the flag is set to false and t_final is resetted */
     flag = false;
}

void sub_pub::sampling_trajectory ( double dt, double t_f )
{
     //Finding the dt and t_f parameters
     if ( !node_handle.getParam ( "trajectory_oo/dt", dt ) || !node_handle.getParam ( "trajectory_oo/t_f", t_f ) ) { 
          ROS_ERROR ( "Could not get parameter dt or t_f !" );
// 	  return 0;
     }

     //Declaring the frequency of messaging
     frequency = 1/dt;
     rate = std::make_shared<ros::Rate>(frequency);


     while ( ( q_d_sym - q_final ).squaredNorm() > toll ) { 

          std::cout<< "Cicle where we are in squaredNORM." << std::endl;

          if ( flag ) {

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
               for ( int i=0; i<q_d_star.size(); i++ ) {
                    states.position[i] = q_d_star[i];
               }

               std::cout<< "Message:  " << states << std::endl;

               //publishing to command node the new q_desired
               pub_q_desired.publish ( states );
	       
	       rate->sleep();

              
          } else {
               dt_hat = 0;
               dt_hat = ros::Time::now().toSec() - t_final;
               flag = true;
          }

     }


}



int main ( int argc, char **argv )
{
     ros::init ( argc,argv, "trajectory_oo" );
     ros::NodeHandle nh;

     double ti;
     double toll;
     //Defining initial time and the tollerance
     ti = 0;
     toll = 0.1;

     ROS_INFO ( "Node active, ready to work!" );
     sub_pub sub_pub ( &nh );

     ros::spin();

     return 0;

}


