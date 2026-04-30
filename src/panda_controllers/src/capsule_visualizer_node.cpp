#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include "utils/thunder_frankino.h"

// rosrun panda_controllers capsule_visualizer_node

struct CapsuleDef
{
    int link_index;
    double radius;
    double length;
    Eigen::Matrix4d T_offset;
};

Eigen::Matrix<double, 7, 1> q0;
bool init_q0 = false;

void jointsCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    if (msg->position.size() >= 7)
    {
        q0 = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(msg->position.data());
        init_q0 = true;
    }
}

std::vector<CapsuleDef> get_frankino_capsules()
{
    std::vector<CapsuleDef> capsules;
    Eigen::Matrix4d T;

    // idx 0
    T << 0, 0, 1, -0.075,
         0, 1, 0, 0,
        -1, 0, 0, 0.06,
         0, 0, 0, 1;
    capsules.push_back({0, 0.065, 0.105, T});

    // idx 1
    T << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -0.1915,
         0, 0, 0, 1;
    capsules.push_back({1, 0.06, 0.283, T});

    // idx 2
    T << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    capsules.push_back({2, 0.06, 0.14, T});

    // idx 3
    T << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -0.145,
         0, 0, 0, 1;
    capsules.push_back({3, 0.06, 0.15, T});

    // idx 4
    T << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    capsules.push_back({4, 0.06, 0.14, T});

    // idx 5
    T << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -0.26,
         0, 0, 0, 1;
    capsules.push_back({5, 0.06, 0.13, T});

    // idx 6
    T << 0.9968, -0.0799, 0, 0,
         0.0799, 0.9968, 0, 0.08,
         0, 0, 1, -0.13,
         0, 0, 0, 1;
    capsules.push_back({5, 0.025, 0.14, T});

    // idx 7
    T << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -0.035,
         0, 0, 0, 1;
    capsules.push_back({6, 0.05, 0.15, T});

    // idx 8
    T << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, -0.10,
         0, 0, 0, 1;
    capsules.push_back({7, 0.045, 0.14, T});

   // idx 9 
    T <<  0.0, -0.7071,  0.7071,  0.01,
          0.0,  0.7071,  0.7071,  0.01,
         -1.0,  0.0,     0.0,    -0.025,
          0.0,  0.0,     0.0,     1.0;
    capsules.push_back({7, 0.04, 0.11, T});

    // // idx 10
    // T << 1, 0, 0, 0,
    //      0, 0, -1, 0,
    //      0, 1, 0, 0.04,
    //      0, 0, 0, 1;
    // capsules.push_back({8, 0.07, 0.10, T});

    // // idx 11
    // T << 1, 0, 0, 0,
    //      0, 0, -1, 0,
    //      0, 1, 0, 0.10,
    //      0, 0, 0, 1;
    // capsules.push_back({8, 0.05, 0.10, T});
    
    return capsules;
}

void publish_capsule_markers(
    thunder_frankino &robot,
    ros::Publisher &marker_pub,
    const std::vector<CapsuleDef> &capsules_definitions)
{
    std::vector<Eigen::Matrix4d> link_poses = {
        robot.get_T_0_0(), robot.get_T_0_1(), robot.get_T_0_2(),
        robot.get_T_0_3(), robot.get_T_0_4(), robot.get_T_0_5(),
        robot.get_T_0_6(), robot.get_T_0_7(), robot.get_T_0_8()};

    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < capsules_definitions.size(); ++i)
    {
        const auto &cap = capsules_definitions[i];

        if (cap.link_index < 0 || cap.link_index >= link_poses.size())
            continue;

        Eigen::Matrix4d T_world_capsule = link_poses[cap.link_index] * cap.T_offset;

        Eigen::Vector3d center = T_world_capsule.block<3, 1>(0, 3);
        Eigen::Vector3d z_axis = T_world_capsule.block<3, 1>(0, 2);
        Eigen::Vector3d half_axis = z_axis * (cap.length / 2.0);

        Eigen::Vector3d a = center - half_axis;
        Eigen::Vector3d b = center + half_axis;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "panda_link0";
        marker.header.stamp = ros::Time::now();
        marker.ns = "collision_capsules";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        Eigen::Vector3d marker_center = (a + b) / 2.0;
        Eigen::Vector3d axis_vector = (b - a).normalized();
        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis_vector);

        marker.pose.position.x = marker_center.x();
        marker.pose.position.y = marker_center.y();
        marker.pose.position.z = marker_center.z();
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();

        marker.scale.x = cap.radius * 2.0;
        marker.scale.y = cap.radius * 2.0;
        marker.scale.z = cap.length;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;

        marker.lifetime = ros::Duration(0.2);

        marker_array.markers.push_back(marker);
    }

    marker_pub.publish(marker_array);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capsule_visualizer");
    ros::NodeHandle nh;

    thunder_frankino robot;
    const std::string conf_file = "../config/frankino_conf.yaml";
    robot.load_conf(conf_file);

    ros::Subscriber sub_joints = nh.subscribe<sensor_msgs::JointState>("/franka/joint_states_1khz", 1, &jointsCallback);

    ros::Publisher pub_capsules = nh.advertise<visualization_msgs::MarkerArray>("/frankino_collision_capsules", 1);

    std::vector<CapsuleDef> my_capsules = get_frankino_capsules();

    ROS_INFO("In attesa dei joint_states per il tracciamento dinamico...");

    // Attendiamo il primo dato valido
    ros::Rate wait_rate(10);
    while (ros::ok() && !init_q0)
    {
        ros::spinOnce();
        wait_rate.sleep();
    }

    ROS_INFO("Tracciamento attivo! Pubblicazione capsule in RViz...");

    // LOOP PRINCIPALE (Frequenza di aggiornamento in RViz)
    // 30 Hz 
    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {
        ros::spinOnce(); // Questo aggiorna q0 con i dati in tempo reale

        // 1. Aggiorna la cinematica interna del robot con le ultimissime posizioni misurate
        robot.set_q(q0);

        // 2. Calcola le nuove trasformate ed emette l'array di cilindri
        publish_capsule_markers(robot, pub_capsules, my_capsules);

        loop_rate.sleep();
    }

    return 0;
}