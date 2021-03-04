
#include <iostream>
#include <eigen3/Eigen/Dense>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>

#include <panda_controllers/DesiredProjectTrajectory.h>
#include <panda_controllers/cubeEq_Preset.h>
#include "utils/parsing_utilities.h"
#include "ros/ros.h"

#include <sstream>
#include <eigen_conversions/eigen_msg.h>

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Pose.h"




using namespace std;

#define alpha 0.1

struct traj_struct{
    Eigen::Vector3d pos_des;
    Eigen::Vector3d vel_des;
    Eigen::Vector3d acc_des;
    Eigen::Vector3d or_des;
    Eigen::Vector3d dor_des;
    Eigen::Vector3d ddor_des;
} traj;

Eigen::Vector3d pos;
Eigen::Vector3d orient;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   exit(signum);
}

void poseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  
  pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  orient << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
}

void interpolator_pos(   Eigen::Vector3d pos_i, Eigen::Vector3d pos_f,
                            double tf, double t, 
                            Eigen::Vector3d vel){

    double ta = alpha*tf;                       // time interval of acceleration
    Eigen::Vector3d xa;
    xa << (1/2)*vel*ta;          // space done during the acceleration
    
    if ((t >= 0) && (t < ta)){
        
        traj.acc_des << vel/ta;
        traj.vel_des << traj.acc_des*t;
        traj.pos_des << pos_i + (1/2)*traj.acc_des*pow(t,2);
    }
    else if ((t >= ta) && (t < (tf-ta))){
        traj.acc_des << 0, 0, 0; 
        traj.vel_des << vel; 
        traj.pos_des << pos_i + xa + traj.vel_des*(t-ta);
    }
    else if ((t >= (tf- ta)) && (t < tf)){
        
        traj.acc_des << -vel/ta;
        traj.vel_des << vel + traj.acc_des*(t - (tf-ta));
        traj.pos_des << pos_i + xa + vel*(tf-2*ta) + vel*(t-(tf-ta)) + (1/2)*traj.acc_des*pow((t-(tf-ta)),2);
    }
    else {
        traj.acc_des << 0, 0, 0;
        traj.vel_des << 0, 0, 0;
        traj.pos_des << pos_f;
    }
}

void demo_inf_XY(Eigen::Vector3d pos_i, double t){
    Eigen::Vector3d tmp;
    tmp << sin(t)/8, sin(t/2)/4, 0;
    traj.pos_des << pos_i + tmp;
    traj.vel_des << cos(t)/8, cos(t/2)/8, 0;
    traj.acc_des << -sin(t)/8, -sin(t/2)/16, 0;
    
}

void demo_inf_XYZ(Eigen::Vector3d pos_i, double t,double zf,double tf){
    Eigen::Vector3d tmp;
    tmp << sin(t)/8, sin(t/2)/4, pos_i(2) + ((zf-pos_i(2))/tf)*t;
    traj.pos_des << pos_i + tmp;
    traj.vel_des << cos(t)/8, cos(t/2)/8, (zf-pos_i(2))/tf;
    traj.acc_des << -sin(t)/8, -sin(t/2)/16, 0;
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "proj_trajectory");

  ros::NodeHandle node_handle;

  ros::Publisher pub_cmd = node_handle.advertise<panda_controllers::DesiredProjectTrajectory>("/project_impedance_controller/desired_project_trajectory", 1000);
  
  ros::Publisher pub_cube = node_handle.advertise<panda_controllers::cubeEq_Preset>("/qb_class/cube_ref",1000);

  ros::Subscriber sub_cmd =  node_handle.subscribe("/project_impedance_controller/franka_ee_pose", 1, 
                                                &poseCallback);

  ros::Rate loop_rate(100);

  panda_controllers::DesiredProjectTrajectory traj_msg;
  panda_controllers::cubeEq_Preset cube_msg;
   

  Eigen::Vector3d pos_f;
  Eigen::Vector3d or_f;
  double tf;
  double zf;
  Eigen::Vector3d vel;
  Eigen::Vector3d pos_init;
  Eigen::Vector3d or_init;
  int inter_x, inter_y, inter_z, comp_x, comp_y, comp_z;
  XmlRpc::XmlRpcValue traj_par;
  // std::map<std::string, std::vector<double>>  traj_par; 

  int N_ACTION;
  Eigen::MatrixXd ACTIONS;
  Eigen::MatrixXi TYPE;

  signal(SIGINT, signal_callback_handler);

  ros::Time t_init;
  double t = 0;
  int choice;
  int demo = -1;
  int yaml = 0;
  int n_act = 0;
  std::vector<float> cube_eq;
  std::vector<float> CUBE_RS;
  // float cube_eq;    
  // float CUBE_RS;


  while (ros::ok()){

    demo = -1;
    if (yaml==1){
      choice = 5;
    }else{
      cout<<"choice:   (1:position , 2:orientation , 3:int-comp, 4:demos, 5:yaml) "<<endl;
      cin>>choice;
    }
    
    if (choice == 1){
      cout<<"time_f "<<endl;
      cin>>tf;
      cout<<"final_position "<<endl;
      cin>> pos_f.x();
      cin>> pos_f.y();
      cin>> pos_f.z();
    }else if (choice == 2){
      cout<<"time_f "<<endl;
      cin>>tf;
      cout<<"final_orientation "<<endl;
      cin>> or_f.x();
      cin>> or_f.y();
      cin>> or_f.z();
      cout<<"sorry, not implemented yet"<<endl;
    }else if (choice == 3){
      tf = -1;
      cout<<"interaction: "<<endl;
      cin>>inter_x;
      cin>>inter_y;
      cin>>inter_z;
      cout<<"compensation: "<<endl;
      cin>>comp_x;
      cin>>comp_y;
      cin>>comp_z;
      cout << "done!"<<endl;
    }else if (choice == 4){
      cout<<"select demo:   (1: infinite XY , 2: infinite XYZ , ...-soon other demos-"<<endl;
      cin>>demo;
      cout<<"insert time_f: "<<endl;
      cin>>tf;
      if (demo==2){
        cout<<"insert zf: "<<endl;
        cin>>zf;
      }
    }else if (choice == 5){
      if (yaml==0){
        yaml = 1;
        if(!node_handle.getParam("/traj_par", traj_par)){
          ROS_ERROR("Could not get the XmlRpc value.");
        }
        if(!parseParameter(traj_par, N_ACTION, "N_ACTION")){
          ROS_ERROR("Could not parse traj_par.");
        }
        if(!parseParameter(traj_par, ACTIONS, "ACTIONS")){
          ROS_ERROR("Could not parse traj_par.");
        }
        if(!parseParameter(traj_par, TYPE, "TYPE")){
          ROS_ERROR("Could not parse traj_par."); 
        }
        // if(!parseParameter(traj_par, CUBE_RS, "CUBE_RS")){
        //   ROS_ERROR("Could not parse traj_par."); 
        // }
        if(!node_handle.getParam("/traj_par/CUBE_RS", CUBE_RS[0])){
          ROS_ERROR("Could not get the XmlRpc value.");
        }
        
      }else{
        if(n_act == N_ACTION){
          yaml = 0;
        }else{
          tf = ACTIONS(n_act,0);
          pos_f(0) = ACTIONS(n_act,1);
          pos_f(1) = ACTIONS(n_act,2);
          pos_f(2) = ACTIONS(n_act,3);
          inter_x = TYPE(n_act,0);
          inter_y = TYPE(n_act,1);
          inter_z = TYPE(n_act,2);
          comp_x = TYPE(n_act,3);
          comp_y = TYPE(n_act,4);
          comp_z = TYPE(n_act,5);
          cube_eq[0] = ACTIONS(n_act,4);
          choice = 1;
          n_act++;
        }
      }

    }

    ros::spinOnce();

    t_init = ros::Time::now();
    pos_init = pos;
    or_init = orient;
    vel << (pos_f.x() - pos_init.x())/((1-alpha)*tf), (pos_f.y() - pos_init.y())/((1-alpha)*tf), (pos_f.z() - pos_init.z())/((1-alpha)*tf);

    t = (ros::Time::now() - t_init).toSec();

    while (t <= tf)
    {
      if (choice == 1){
        interpolator_pos(pos_init, pos_f, tf, t, vel);
      } else if (choice == 2){
        //interpolator_or(or_init, or_f, tf, t, vel_or)
      } else if (choice == 4){
        if (demo == 1){
          demo_inf_XY(pos_init, t);
        }else if(demo==2){
          demo_inf_XYZ(pos_init,t,zf,tf);
        }
      }

      traj_msg.header.stamp = ros::Time::now();

      traj_msg.pose.position.x = traj.pos_des.x();
      traj_msg.pose.position.y = traj.pos_des.y();
      traj_msg.pose.position.z = traj.pos_des.z();
      traj_msg.pose.orientation.x = orient.x();
      traj_msg.pose.orientation.y = orient.y();
      traj_msg.pose.orientation.z = orient.z();
      
      traj_msg.velocity.position.x = traj.vel_des.x();
      traj_msg.velocity.position.y = traj.vel_des.y();
      traj_msg.velocity.position.z = traj.vel_des.z();
      traj_msg.velocity.orientation.x = 0;
      traj_msg.velocity.orientation.y = 0;
      traj_msg.velocity.orientation.z = 0;

      traj_msg.acceleration.position.x = traj.acc_des.x();
      traj_msg.acceleration.position.y = traj.acc_des.y();
      traj_msg.acceleration.position.z = traj.acc_des.z();
      traj_msg.acceleration.orientation.x = 0;
      traj_msg.acceleration.orientation.y = 0;
      traj_msg.acceleration.orientation.z = 0;

      traj_msg.interaction[0] = inter_x;
      traj_msg.interaction[1] = inter_y;
      traj_msg.interaction[2] = inter_z;
      traj_msg.compensation[0] = comp_x;
      traj_msg.compensation[1] = comp_y;
      traj_msg.compensation[2] = comp_z;

      pub_cmd.publish(traj_msg);

      cube_msg.eq = cube_eq;
      cube_msg.preset = CUBE_RS;
      pub_cube.publish(cube_msg);

      loop_rate.sleep();

      t = (ros::Time::now() - t_init).toSec();
    }
  }
  return 0;
}
