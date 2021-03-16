/*
    Authors: 
        Chiara Sammarco
        Rachele Nebbia Colomba
        Giorgio Simonini
    Description: 
        ROS node containing the planner for a variable impedance controller

*/

#include "panda_controllers/planner.h"

#define     F_MAX       4.0         // [N]          disturbance threshold
#define     E_MAX       0.05        // [m]          maximum tollerated error 
#define     F_INT_MAX   7.0         // [N]          maximum tollerated force in interaction
#define     K_MIN       10.0        // [N/m]        minimum value for stiffness
#define     K_MAX       1000.0      // [N/m]        maximum value for stiffness
#define     MASS        1.0         // [kg]         virtual mass (inertia shaping)
#define     BETA        0.98        // []           <1 due to stability
#define     A0          0.99        // []           <1 due to stability
#define     CSI         1.0         // []           critically damped system
#define     K_OR        500         // [Nm/rad]     orientation stiffness
#define     D_OR        2*sqrt(500) // [Nm*sec/rad] orientation damping
#define     K_INIT      200         // [Nm]         default value
#define     K_INIT_X      500         // [Nm]         default value x
#define     K_INIT_Y      500         // [Nm]         default value y
#define     K_INIT_Z      100         // [Nm]         default value z

//==========================================================================================//
//                                      CLASS PLANNER                                       //
//==========================================================================================//
planner_class::planner_class(){
    F_comp = 0;
    z_int = 1000;
    z_int_dir = 0;
    int_prec = 0;
    comp_prec = 0;
    set_F_comp = 1;
}

void planner_class::set_k_init(double k){
    ki = k;
    kc = k;
    k_init = k;
}

int planner_class::sign(double x){
    float tmp = x/std::abs(x);
    if(tmp<0)
        return -1;
    else
        return 1;
}

void planner_class::get_info(int axis, double *z_int_, int *z_int_dir_, int *set_F_comp_, double *ki_, double *kc_){
    z_int_[axis] = z_int;
    z_int_dir_[axis] = z_int_dir;
    set_F_comp_[axis] = set_F_comp;
    ki_[axis] = ki;
    kc_[axis] = kc;
}

double planner_class::planning(double F_max, double e_max, double F_int_max, double F_ext, double z, double z_des, double dz_des, int inter, int comp){

    // INTERACTION
    // set ki in order to match a desired interaction force with the
    // envirorment, and store interaction's position when detected
    if (inter == 1){
            // std::cout << " F_comp: " << F_comp << std::endl;
            // std::cout << " F_ext: " << F_ext << std::endl;
        // detection of an interaction 
        if (std::abs(F_ext-F_comp) > F_MAX){
            // std::cout << " F_ext > F_max" << std::endl;
            if (z_int > 999 ){                     // z_int has not been set yet
                z_int = z;
                z_int_dir = -sign(F_ext-F_comp);
                set_F_comp = 0; 
            }
        }
        // std::cout << "z_int_dir is: " << z_int_dir << std::endl;
        // std::cout << "z_int is: " << z_int << std::endl;
        // setting of ki
        if (std::abs(F_ext-F_comp) > F_int_max){
            // std::cout << "F_ext > F_int_max" << std::endl;
            // if interaction force is higher than threshold
            if (ki*std::abs(z-z_des) > F_int_max){
                double ktemp = F_int_max/std::abs(z_int-z_des);
                // std::cout << "ktemp: " << ktemp << std::endl;
                // std::cout << "z_des: " << z_des << std::endl;
                // std::cout << "ki: " << ki << std::endl;
                if (ktemp < ki){                // ki needs to be only decreased
                    ki = ktemp;
                }
            }else{
                if (ki > K_MIN){
                    ki = 0.995*ki;              // designed for planning 100Hz
                }else{
                    ki = K_MIN;
                }
            }
        }
    }

    // COMPENSATION
    // set kc in order to compensate the effect of the weight of the object,
    // and store it
    if (comp == 1){
        // compensation detection
        if (z_int > 999){                     // z_int not set before
            if (std::abs(F_ext)/kc > e_max){         
                kc = std::abs(F_ext)/e_max;
                if (set_F_comp == 1){
                    F_comp = F_ext;
                }
            }
        }else{ //maybe not needed anymore, because z_int acknowledges the "still in contact" condition
            if (z_int_dir == 1){                // obstacle in above
                if (z < z_int){
                    if (std::abs(F_ext)/kc > e_max){ 
                        kc = std::abs(F_ext)/e_max;
                        if (set_F_comp == 1){
                            F_comp = F_ext;
                        }
                    }
                }
            }else{                                // obstacle is below
                if (z > z_int){
                    if (std::abs(F_ext)/kc > e_max){
                        kc = std::abs(F_ext)/e_max;
                        if (set_F_comp == 1){
                            F_comp = F_ext;
                        }
                    }
                }
            } 
        }
    }

    // reset to default values when comp or int shifts to zero
    if (comp==0 && comp_prec==1){
        kc = k_init;
        F_comp = 0;
        set_F_comp = 0;
    }
    if (inter==0 && int_prec==1){
        ki = k_init;
    }

    // from int to comp and vice versa
    if (inter==1 && int_prec==0){
        if (comp_prec == 1){
            ki = kc;
        }
        z_int = 1000;
        z_int_dir = 0;
    }
    if (comp==1 && comp_prec==0){
        if (int_prec == 1){
            kc = ki;
        }
        set_F_comp = 1;
    }

    // update to last booleans
    int_prec = inter;
    comp_prec = comp;


    // Selection of Final Values of D-K
    // default values
    double k = k_init;

    // compensation
    if (comp == 1){
        k = kc;
    }

    // both compensation and interaction are activated
    // if "away" from obstacle set k = kc else ki
    if (inter == 1){
        k = ki;
        // std::cout <<"ki_sel = " << ki << std::endl;
        if (comp == 1){
            if (z_int > 999){
                k = kc;   
                // std::cout<<" not ok 1 " << std::endl;                    // obstacle not reached
            }else{
                if (z_int_dir == 1){
                    if (z_des < z_int){
                        if (sign(dz_des*z_int_dir) == -1){
                            k = kc;             // obstacle reached but "going away"

                        }
                    }
                }else{
                    if (z_des > z_int){
                        if (sign(dz_des*z_int_dir) == -1){
                            k = kc;            // obstacle reached but "going away"
                        }
                    }
                }
            }
        }else{
        //    std::cout <<"ok1 "<<std::endl;
            if (z_int > 999){
                k = k_init;            // restore to default
                // std::cout <<"not ok1 " << std::endl;
            }else{
        //        std::cout <<"ok2 "<<std::endl;
                if (z_int_dir == 1){
                    // std::cout <<"not ok" << std::endl;
                    if (z_des < z_int){
                        // std::cout <<"not ok2 " << std::endl;
                        k = k_init;  // restore because "going away"
                    }
                }else{
                    // std::cout<<" z_int_dir != 1 "<<std::endl;
                    if (z_des > z_int){
                        // std::cout<<"not ok3 "<<std::endl;
                        k = k_init;  // restore because "going away"
                    }
                }
            }
        }
    }
    // std::cout<<"z_int "<< z_int << std::endl;
    // std::cout<<"k "<< k << std::endl;
    return k;
}



//==========================================================================================//
//                                      NODE PLANNER                                        //
//==========================================================================================//

planner_node::planner_node(){
    for(int i=0;i<36;i++){
        K[i] = 0;
        D[i] = 0;
    }
    time_prec = ros::Time::now();
    kx = K_INIT_X;
    ky = K_INIT_Y;
    kz = K_INIT_Z;
    planner_x.set_k_init(kx);
    planner_y.set_k_init(ky);
    planner_z.set_k_init(kz);
    // kx = F_MAX/E_MAX;
    // ky = F_MAX/E_MAX;
    // kz = F_MAX/E_MAX;
}


//------------------------------------------------------------------//
//                              INIT                                //
//------------------------------------------------------------------//
bool planner_node::init(ros::NodeHandle& node_handle){

  //---------------SUBSCRIBERS AND PUBLISHERS---------------//

  sub_des_traj_proj_ = node_handle.subscribe(
      "/project_impedance_controller/desired_project_trajectory", 1, &planner_node::desiredProjectTrajectoryCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_ee_pose = node_handle.subscribe(
      "/project_impedance_controller/franka_ee_pose", 1, &planner_node::ee_pose_Callback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_ext_forces = node_handle.subscribe(
      "/project_impedance_controller/ext_forces", 1, &planner_node::f_ext_Callback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  

  pub_impedance = node_handle.advertise<panda_controllers::DesiredImpedance>("/project_impedance_controller/desired_impedance_project", 1);
  

  //---------------INITIALIZE VARIABLES---------------//
   F_ext.setZero();                      
   pos_d.setZero();                      
   dpos_d.setZero();
   interaction.setZero();
//    interaction(0)=1;
//    interaction(1)=1;
//    interaction(2)=1;
   compensation.setZero();
//------------------------------------------------------------------------------------------------REMOVE THIS!
//    compensation(0)=1;
//    compensation(1)=1;
//    compensation(2)=1;
//---------------------------end
  return true;
}


//------------------------------------------------------------------//
//                             UPDATE                               //
//------------------------------------------------------------------//
void planner_node::update() {

    //---------------PLANNING---------------//

    double kx_f = planner_x.planning(F_MAX, E_MAX, F_INT_MAX, F_ext(0), ee_pos(0), pos_d(0), dpos_d(0), interaction(0), compensation(0));
    double ky_f = planner_y.planning(F_MAX, E_MAX, F_INT_MAX, F_ext(1), ee_pos(1), pos_d(1), dpos_d(1), interaction(1), compensation(1));
    double kz_f = planner_z.planning(F_MAX, E_MAX, F_INT_MAX, F_ext(2) , ee_pos(2), pos_d(2), dpos_d(2), interaction(2), compensation(2));

    double z_int_[3];
    int z_int_dir[3];
    int set_F_comp_[3];
    double ki_[3];
    double kc_[3];
    planner_x.get_info(0, z_int_, z_int_dir_, set_F_comp_, ki_, kc_);
    planner_y.get_info(1, z_int_, z_int_dir_, set_F_comp_, ki_, kc_);
    planner_z.get_info(2, z_int_, z_int_dir_, set_F_comp_, ki_, kc_);

    // std::cout << "kz_f: " << kz_f << std::endl;

    // std::cout << "Fx " << F_ext(0) << std::endl;
    // std::cout << "Fy " << F_ext(1) << std::endl;
    // std::cout << "Fz " << F_ext(2) << std::endl;

    // double kx_f = 200;
    // double ky_f = 200;
    interpolator(kx_f,ky_f,kz_f);
    

//------------------------------------------------------------------------------------------------REMOVE THIS!
    //std::cout << "x_real: " << ee_pos(0) << "  x_des: " << pos_d(0) << std::endl;
    //std::cout << "y_real: " << ee_pos(1) << "  y_des: " << pos_d(1) << std::endl;
    //std::cout << "z_real: " << ee_pos(2) << "  z_des: " << pos_d(2) << std::endl;
//------------------------end

    //---------------PUBLISHING----------------//

    desired_impedance_msg.header.stamp = ros::Time::now();

    for ( int i = 0; i <36; i++){
        desired_impedance_msg.stiffness_matrix[i] = K[i];
        desired_impedance_msg.damping_matrix[i] = D[i];
    }
    // std::cout << "K[0]" << K[0] << ", Kmsg" << desired_impedance_msg.stiffness_matrix[0] << std::endl;
    pub_impedance.publish(desired_impedance_msg);


    info_planner_msg.header.stamp = ros::Time::now();
    for (int i = 0; i<3; i++){
        info_planner_msg.z_int[i] = z_int_[i];
        info_planner_msg.z_int_dir[i] = z_int_dir_[i];
        info_planner_msg.F_comp[i] = F_comp_[i];
        info_planner_msg.set_F_comp[i] = set_F_comp_[i];
        info_planner_msg.ki[i] = ki_[i];
        info_planner_msg.kc[i] = kc_[i];
    }
    pub_info_planner.publish(info_planner_msg)

}
// mettere topic

//------------------------------------------------------------------//
//                          INTERPOLATOR                            //
//------------------------------------------------------------------//
void planner_node::interpolator(double kx_f, double ky_f, double kz_f){

    ros::Time time = ros::Time::now();

    kx = calc_k(kx, kx_f);
    ky = calc_k(ky, ky_f);
    kz = calc_k(kz, kz_f);

    time_prec = time;

    // damping computation, critically damped condition
    double dx = sqrt(4*kx*MASS);
    double dy = sqrt(4*ky*MASS);
    double dz = sqrt(4*kz*MASS);

    // Final impedance matrices
    K[0] = kx;
    K[7] = ky;
    K[14] = kz;
    K[21] = K_OR;
    K[28] = K_OR;
    K[35] = K_OR;

    // std::cout << "kx: " << kx << std::endl;
    // std::cout << "ky: " << ky << std::endl;
    // std::cout << "kz: " << kz << std::endl;

    D[0] = dx;
    D[7] = dy;
    D[14] = dz;
    D[21] = D_OR;
    D[28] = D_OR;
    D[35] = D_OR;
}


//------------------------------------------------------------------//
//                             CALK_K                               //
//------------------------------------------------------------------//
// Compute K taking into account stability
double planner_node::calc_k(double k, double k_f){
    // set k from desired one 
    ros::Time time = ros::Time::now();
    //double interval = time.toSec() - time_prec.toSec();
    double interval = (time - time_prec).toSec();
    if (k_f <= k){
        if (k_f < K_MIN)
            k = K_MIN;
        else
            k = k_f;
    }else{
        // increase k in accordance with the stability conditions
        k_f = (k_f > K_MAX) ? K_MAX : k_f;
        double k_i = k;
        double k_dot = BETA*(4*A0*sqrt(k_i/MASS)*pow(k_i,3/2))/(sqrt(k_i) + 2*A0*CSI*sqrt(k_i));
        double k_temp = k_i + k_dot*interval;
        if (k_temp > k_f){
            k = k_f;
        }else{
            k = k_temp;
        }
    }
    return k;
}



//------------------------------------------------------------------//
//                           CALLBACKS                              //
//------------------------------------------------------------------//
void planner_node::ee_pose_Callback(const geometry_msgs::PoseStampedConstPtr& msg) {
  
    ee_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

void planner_node::desiredProjectTrajectoryCallback(const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
    
    pos_d << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    dpos_d << msg->velocity.position.x, msg->velocity.position.y, msg->velocity.position.z;
    interaction << msg->interaction[0], msg->interaction[1], msg->interaction[2];
    compensation << msg->compensation[0], msg->compensation[1], msg->compensation[2];
}

void planner_node::f_ext_Callback(const geometry_msgs::WrenchStampedConstPtr& msg){
  F_ext << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
}



//==========================================================================================//
//                                         MAIN                                             //
//==========================================================================================//
int main(int argc, char **argv){
    
    ros::init(argc, argv, "planner");
    ros::NodeHandle node_planner;

    ros::Rate loop_rate(200); //planner's frequency Hz
    
    planner_node planner;
    planner.init(node_planner);

    while (ros::ok()){
        ros::spinOnce();
        
        planner.update();

        loop_rate.sleep();
    }
    return 0;
}
