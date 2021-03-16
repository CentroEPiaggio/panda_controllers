/*
    Authors: 
        Chiara Sammarco
        Rachele Nebbia Colomba
        Giorgio Simonini
    Description: 
        ROS node containing the planner for a variable impedance controller
        mk2 version
    To Do:
        rename to planner and substitute the other

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
    kf = k;
    k_init = k;
}

int planner_class::sign(double x){
    float tmp = x/std::abs(x);
    if(tmp<0)
        return -1;
    else
        return 1;
}

double planner_class::planning(double F_max, double e_max, double F_int_max, double F_ext, double z, double z_des, double dz_des, int inter, int comp){

    // insert planning code from matlab--------------------------------------------------------------------------  <=======
    /*
__________uu$$$$$$$$$$$$$$$$$uu__________
_________u$$$$$$$$$$$$$$$$$$$$$u_________
________u$$$$$$$$$$$$$$$$$$$$$$$u________
_______u$$$$$$$$$$$$$$$$$$$$$$$$$u_______
_______u$$$$$$$$$$$$$$$$$$$$$$$$$u_______
_______u$$$$$$”___”$$$”___”$$$$$$u_______
_______”$$$$”______u$u_______$$$$”_______
________$$$———u$u_______u$$$__$$$________
________$$$u______u$$$u______u$$$________
_________”$$$$uu$$$___$$$uu$$$$”_________
__________”$$$$$$$”___”$$$$$$$”__________
____________u$$$$$$$u$$$$$$$u____________
_____________u$”$”$”$”$”$”$u_____________
__uuu________$$u$_$_$_$_$u$$_______uuu___
_u$$$$________$$$$$u$u$u$$$_______u$$$$__
__$$$$$uu______”$$$$$$$$$”_____uu$$$$$$__
u$$$$$$$$$$$uu____”””””____uuuu$$$$$$$$$$
$$$$”””$$$$$$$$$$uuu___uu$$$$$$$$$”””$$$”
_”””______””$$$$$$$$$$$uu_””$”””_________
___________uuuu_””$$$$$$$$$$uuu__________
__u$$$uuu$$$$$$$$$uu_””$$$$$$$$$$$uuu$$$_
__$$$$$$$$$$””””___________””$$$$$$$$$$$”
___”$$$$$”______________________””$$$$””_
*/

    function k = fcn(F_ext, z, z_des, dz_des, int, comp, e_max, F_max, F_int_max)
    %% Description
    % Set the K coefficient for a variable impedance controller based on a
    % desired trajectory

    %% Outputs
    % k             stiffness

    %% Inputs
    % F_ext         measured external force
    % z_des         desired trajectory
    % z             current position
    % int, comp     booleans for the interaction and compensation phase

    %% Parameters
    % F_int_max     desired interaction force
    % F_max         nominal disturbance forces
    % e_max         maximum steady-state error

    %% persistent initialization
    K_INIT = 200;
    K_MIN = 10;
    K_MAX = 1000;
    persistent F_comp int_prec comp_prec z_int z_int_dir set_F_comp K
    if isempty(F_comp)
    F_comp = 0;              % weight to compensate
    end
    if isempty(z_int) 
        z_int = 1000;       % obstacle(or interaction) position
    end
    if isempty(z_int_dir) 
        z_int_dir = 0;          % prohibited direction due to the obstacle
    end
    if isempty(int_prec)
        int_prec = 0;           % previous values of int
    end
    if isempty(comp_prec)
        comp_prec = 0;          % previous values od comp
    end
    if isempty(set_F_comp)
        set_F_comp = 1;
    end
    if isempty(K)
        K = F_max/e_max;
    end

    %% Detection
    % set k in order to match a desired interaction force with the
    % envirorment, and store interaction's position when detected
    % X axes
    if int == 1
        % detection of an interaction 
        if abs(F_ext-F_comp) > F_max
            if z_int == 1000                     % z_int has not been set yet
                z_int = z;
                z_int_dir = -sign(F_ext-F_comp);
                set_F_comp = 0; 
            end
        end
    end

    %% Going away
    if z_int < 999
        if z_int_dir == 1
            if z_des < z_int
                z_int = 1000;       % going away
                set_F_comp = 1;
            end
        else
            if z_des > z_int
                z_int = 1000;       % going away
                set_F_comp = 1;
            end
        end
    end
        
    %% Computation
    if int == 0 
        if comp == 0
            K = K_INIT;
        else
            if abs(F_ext)/K > e_max         
                K = abs(F_ext)/e_max;
                if set_F_comp > 0.1
                    F_comp = F_ext;
                end
            end
        end
    end

    if int == 1
        if comp == 0
            if abs(F_ext) > F_int_max 
                % if interaction force is higher than threshold
                if K*abs(z-z_des) > F_int_max
                    ktemp = F_int_max/abs(z-z_des);
                    if ktemp < K                % K needs to be only decreased
                        K = ktemp;
                    end
                    
                else
                    K = 0.995*K;
                end
                if K < K_MIN
                    K = K_MIN;
                end
            end
        else
            if z_int > 999
                if abs(F_ext)/K > e_max         
                    K = abs(F_ext)/e_max;
                    if set_F_comp > 0.1
                        F_comp = F_ext;
                    end
                    if K > K_MAX
                        K = K_MAX;
                    end
                end
            else
                if abs(F_ext-F_comp) > F_int_max 
                    % if interaction force is higher than threshold
                    if K*abs(z-z_des) > F_int_max
                        ktemp = F_int_max/abs(z-z_des);
                        if ktemp < K                % K needs to be only decreased
                            K = ktemp;
                        end
                    else
                        K = 0.995*K;
                    end
                    if K < K_MIN
                        K = K_MIN;
                    end
                end
            end
        end
    end

    %% Reset to defaults
    if comp==0 && comp_prec==1
    %     kc = F_max/e_max;
        F_comp = 0;
        set_F_comp = 0;
    end
    if int==0 && int_prec==1
    %     ki = F_max/e_max;
    end

    %% from int to comp and vice versa
    if int==1 && int_prec==0
        z_int=1000;
        z_int_dir=0;
    end
    if comp == 1 && comp_prec==0
        set_F_comp = 1;
    end

    % update to last booleans
    int_prec = int;
    comp_prec = comp;

    k=K;

    end


    return kf;
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
}


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
