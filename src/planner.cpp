/*
    Authors: 
        Chiara Sammarco
        Rachele Nebbia Colomba
        Giorgio Simonini
    Description: 
        ROS node containing the planner for a variable impedance controller

*/

#include "panda_controllers/planner.h"

#define     F_MAX       2.0         // [N]
#define     E_MAX       0.01        // [m]
#define     F_INT_MAX   5.0         // [N]
#define     MASS        1.0         // [kg]
#define     BETA        0.98        // []
#define     A0          0.99        // []
#define     CSI         1.0         // []
#define     K_OR        300         // [Nm/rad]
#define     D_OR        2*sqrt(300) // [Nm*sec/rad]


//----------------------------------------------------------//
//                      PLANNER_CLASS                       //
//----------------------------------------------------------//
planner_class::planner_class(){
    ki = F_MAX/E_MAX;
    kc = F_MAX/E_MAX;
    F_comp = 0;
    z_int = 1000;
    z_int_dir = 0;
    int_prec = 0;
    comp_prec = 0;
    set_F_comp = 1;
}

int planner_class::sign(double x){
    float tmp = x/fabs(x);
    if(tmp<0)
        return -1;
    else
        return 1;
}

double planner_class::planning(double F_max, double e_max, double F_int_max, double F_ext, double z, double z_des, double dz_des, int inter, int comp){

    // INTERACTION
    // set ki in order to match a desired interaction force with the
    // envirorment, and store interaction's position when detected
    // X axes
    if (inter == 1){
        // detection of an interaction 
        if (fabs(F_ext-F_comp) > F_MAX){
            if (z_int > 999 ){                     // z_int has not been set yet
                z_int = z;
                z_int_dir = -sign(F_ext-F_comp);
                set_F_comp = 0; 
            }
        }
        // setting of ki
        if (fabs(F_ext-F_comp) > F_int_max){
            // if interaction force is higher than threshold
            if (ki*fabs(z-z_des) > F_int_max){
                double ktemp_x = F_int_max/fabs(z-z_des);
                if (ktemp_x < ki){                // ki needs to be only decreased
                    ki = ktemp_x;
                }
            }
        }
    }

    // COMPENSATION
    // set kc in order to compensate the effect of the weight of the object,
    // and store it
    // X axes
    if (comp == 1){
        // compensation detection
        if (z_int > 999){                     // z_int not set before
            if (fabs(F_ext)/kc > e_max){         
                kc = fabs(F_ext)/e_max;
                if (set_F_comp == 1){
                    F_comp = F_ext;
                }
            }
        }else{
            if (z_int_dir == 1){                // obstacle in above
                if (z < z_int){
                    if (fabs(F_ext)/kc > e_max){ 
                        kc = fabs(F_ext)/e_max;
                        if (set_F_comp == 1){
                            F_comp = F_ext;
                        }
                    }
                }
            }else{                                // obstacle is below
                if (z > z_int){
                    if (fabs(F_ext)/kc > e_max){
                        kc = fabs(F_ext)/e_max;
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
        kc = F_max/e_max;
        F_comp = 0;
        set_F_comp = 0;
    }
    if (inter==0 && int_prec==1){
        ki = F_max/e_max;
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
    double k = fabs(F_max)/e_max;

    // compensation
    if (comp == 1){
        k = kc;
    }

    // both compensation and interaction are activated
    // if "away" from obstacle set k = kc else ki
    if (inter == 1){
        k = ki;
        if (comp == 1){
            if (z_int > 999){
                k = kc;                       // obstacle not reached
            }else{
                if (z_int_dir == 1){
                    if (z_des < z_int){
                        if (sign(dz_des*z_int_dir) == -1){
                            k = kc;               // obstacle reached but "going away"
                        }
                    }
                }else{
                    if (z_des > z_int){
                        if (sign(dz_des*z_int_dir) == -1){
                            k = kc;               // obstacle reached but "going away"
                        }
                    }
                }
            }
        }else{
            if (z_int > 999){
                k = F_max/e_max;            // restore to default
            }else{
                if (z_int_dir == 1){
                    if (z_des < z_int){
                        k = F_max/e_max;    // restore because "going away"
                    }
                }else{
                    if (z_des > z_int){
                        k = F_max/e_max;    // restore because "going away"
                    }
                }
            }
        }
    }
    return k;
}




//----------------------------------------------------------//
//                      PLANNER_NODE                        //
//----------------------------------------------------------//
planner_node::planner_node(){
    for(int i=0;i<36;i++){
        K[i] = 0;
        D[i] = 0;
    }
    time_prec = ros::Time::now();
}
//----------------------------------------------------------//
bool planner_node::init(ros::NodeHandle& node_handle){
                      
  // Name space extraction for add a prefix to the topic name
  int n = 0;
  std::string name_space;
  name_space = node_handle.getNamespace();
  n = name_space.find("/", 2);
  name_space = name_space.substr(0,n);

 
  //----------------INITIALIZE SUBSCRIBERS AND PUBLISHERS------//

  sub_des_traj_proj_ = node_handle.subscribe(
      name_space+"/desired_project_trajectory", 10, &planner_node::desiredProjectTrajectoryCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_ee_pose = node_handle.subscribe(
      name_space+"/ee_pose", 1, &planner_node::ee_pose_Callback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_ext_forces = node_handle.subscribe(
      name_space+"/f_ext", 1, &planner_node::f_ext_Callback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  

  pub_impedance = node_handle.advertise<panda_controllers::DesiredImpedance>(name_space+"/desired_impedance_project", 1);
  

  /*-------------------INITIALIZE VARIABLES---------------*/
   F_ext.setZero();                      
   pos_d.setZero();                      
   dpos_d.setZero();
   interaction.setZero();
   compensation.setZero();

  return true;
}

void planner_node::update() {
    double kx_f = planner_x.planning(F_MAX, E_MAX, F_INT_MAX, F_ext(0), ee_pos(0), pos_d(0), dpos_d(0), interaction(0), compensation(0));
    double ky_f = planner_y.planning(F_MAX, E_MAX, F_INT_MAX, F_ext(1), ee_pos(1), pos_d(1), dpos_d(1), interaction(1), compensation(1));
    double kz_f = planner_z.planning(F_MAX, E_MAX, F_INT_MAX, F_ext(2), ee_pos(2), pos_d(2), dpos_d(2), interaction(2), compensation(2));
    interpolator(kx_f,ky_f,kz_f);

    for ( int i = 0; i <36; i++){
        desired_impedance_msg.stiffness_matrix[i] = K[i];
        desired_impedance_msg.damping_matrix[i] = D[i];
    }

    pub_impedance.publish(desired_impedance_msg);
}



double planner_node::calc_k(double k, double k_f){
    // set k from desired one 
    ros::Time time = ros::Time::now();
    //double interval = time.toSec() - time_prec.toSec();
    double interval = (time - time_prec).toSec();
    if (k_f <= k){
        k = k_f;
    }else{
        // increese k in accordance with the stability conditions
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

    D[0] = dx;
    D[7] = dy;
    D[14] = dz;
    D[21] = D_OR;
    D[28] = D_OR;
    D[35] = D_OR;
}



//-----------------------CALLBACKS-------------------------//
void planner_node::ee_pose_Callback(const panda_controllers::EEposeConstPtr& msg) {
  
    ee_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

void planner_node::desiredProjectTrajectoryCallback(const panda_controllers::DesiredProjectTrajectoryConstPtr& msg) {
    
    pos_d << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    dpos_d << msg->velocity.position.x, msg->velocity.position.y, msg->velocity.position.z;
    interaction << msg->interaction[0], msg->interaction[1], msg->interaction[2];
    compensation << msg->compensation[0], msg->compensation[1], msg->compensation[2];
}

void planner_node::f_ext_Callback(const panda_controllers::ExternalForcesConstPtr& msg){
    F_ext << msg->forces[0], msg->forces[1], msg->forces[2];
}



int main(int argc, char **argv){
    
    ros::init(argc, argv, "planner");
    ros::NodeHandle node_planner;

    ros::Rate loop_rate(100); //planner's frequency Hz
    
    planner_node planner;
    planner.init(node_planner);

    while (ros::ok()){
        planner.update();

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
