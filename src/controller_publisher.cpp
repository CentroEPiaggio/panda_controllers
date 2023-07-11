#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <eigen3/Eigen/Dense>
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include <boost/filesystem.hpp>

#include <fstream>
#include <sstream>
#include <string>

#include <casadi/casadi.hpp>

std::vector<std::vector<double>> ImportCSV(const std::string& filename) {


    std::vector<std::vector<double>> data;
    std::string line;
    std::string absolutePath = boost::filesystem::absolute(filename).string();

    std::cout << "Percorso assoluto del file: " << absolutePath << std::endl;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Impossibile aprire il file " + filename);
    }

    while (std::getline(file, line)) {

        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stod(cell));
        }

        data.push_back(row);
    }

    file.close();

    return data;
}

boost::math::cubic_b_spline<double> Spline_obj(const std::vector<double>& vec, const double time_0, const double time_step) {
    
    int size = vec.size();
    boost::math::cubic_b_spline<double> spline_obj(vec.begin(), vec.end(), time_0, time_step);

    return spline_obj;
}

int main(int argc, char** argv)
{    
    int num_joints = 7;
    double time_0;      // Tempo iniziale
    double time_step;
    double ros_time;
    double rate_f = 10; // Frequenza di pubblicazione dei messaggi (10 Hz)

    ros::init(argc, argv, "controller_publisher");
 
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub;
    ros::Rate rate(rate_f);

    sensor_msgs::JointState joint_state_msg;

    std::vector<double> column;
    std::vector<double> joint_var;
    std::vector<std::vector<double>> matrix;
    std::vector<boost::math::cubic_b_spline<double>> spline_arr;

    Eigen::VectorXd q_arr;
    Eigen::VectorXd dq_arr;
    Eigen::VectorXd ddq_arr;
    
    std::string filename = "/home/yurs/Documents/MATLAB/Robotica/Simulazioni/Sim_reg_compact_new/q_interp.csv";
    matrix = ImportCSV(filename);
    column.resize(matrix.size());

    q_arr.resize(num_joints);
    dq_arr.resize(num_joints);
    ddq_arr.resize(num_joints);

    joint_state_msg.name.resize(num_joints);  // Aggiorna la dimensione del vettore nomi
    joint_state_msg.position.resize(num_joints);
    joint_state_msg.velocity.resize(num_joints);
    joint_state_msg.effort.resize(num_joints);

    joint_var.resize(num_joints*3);
    spline_arr.resize(num_joints*3);

    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/computed_torque_controller/command", 1);
        
    time_0 = (double)ros::Time::now().sec + (double)ros::Time::now().nsec*1e-9;
    time_step = 0.1;
    
    /* if (num_joints*3 != matrix[0].size()){
        throw std::runtime_error("Dimensione csv errata");
    } */
    
    for (int i = 0; i<num_joints*3; i++){

        for (int j = 0; j < matrix.size(); j++){
            column[j] = matrix[i][j];
        }
        
        spline_arr[i] = Spline_obj(column, time_0, time_step);
    }

    joint_state_msg.name[0] = "panda_joint1";
    joint_state_msg.name[1] = "panda_joint2";
    joint_state_msg.name[2] = "panda_joint3";
    joint_state_msg.name[3] = "panda_joint4";
    joint_state_msg.name[4] = "panda_joint5";
    joint_state_msg.name[5] = "panda_joint6";
    joint_state_msg.name[6] = "panda_joint7";
 
    casadi::SX x = casadi::SX::sym("x",1,1);
    casadi::MX a = casadi::MX::ones(7,7);
    std::cout<<a<<std::endl;

    while (ros::ok())
    {
        joint_state_msg.header.stamp = ros::Time::now();  // Imposta il timestamp corrente        
        ros_time = (double)ros::Time::now().sec + (double)ros::Time::now().nsec*1e-9;

        for (int i = 0; i<num_joints*3; i++){
            joint_var[i] = spline_arr[i](ros_time); 
        }

        for (int i = 0; i<num_joints; i++){
            q_arr(i) = joint_var[i];
            dq_arr(i) = joint_var[num_joints+i];
            ddq_arr(i) = joint_var[2*num_joints+i];
        }

        for (int i = 0; i < num_joints; i++) {
            joint_state_msg.position[i] = q_arr[i]*0;
            joint_state_msg.velocity[i] = dq_arr[i]*0;
            joint_state_msg.effort[i] = ddq_arr[i]*0;
        }


        joint_state_msg.position[0] = ros_time*0.1;
        joint_state_msg.position[3] = -M_PI/2;
        joint_state_msg.position[5] = M_PI/2;

        joint_state_pub.publish(joint_state_msg);

        rate.sleep();
    }
        
    return 0;
}
