#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Core>

Eigen::Matrix3d traj_(double t_sym) {
    
    double A = 0.2, a = 0.5;
    double B = 0.2, b = 1;
    double d = M_PI_2;
    double x0 = 0.5, y0 = 0, z0 = 0.4;

    Eigen::Matrix3d trajectory;
    
    trajectory.row(0) << 0+x0, A * std::sin(a * t_sym) + y0, B * std::sin(b * t_sym + d) + z0;
    trajectory.row(1) << 0, A * a * std::cos(a * t_sym), B * b * std::cos(b * t_sym + d);
    trajectory.row(2) << 0, -A * a * a * std::sin(a * t_sym), -B * b * b * std::sin(b * t_sym + d);

    return trajectory;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_visualizer");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("trajectory_markers", 1);

    double t_max = 5000.0; // Valore massimo di t_sym
    double t_step = 0.1; // Passo di campionamento per t_sym

    visualization_msgs::Marker marker;
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02; // Spessore delle linee
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    for (double t_sym = 0; t_sym <= t_max; t_sym += t_step) {
        Eigen::Matrix3d point = traj_(t_sym);

        // Aggiungi il punto corrente della traiettoria alla linea
        geometry_msgs::Point p;
        p.x = point(0, 0);
        p.y = point(0, 1);
        p.z = point(0, 2);
        marker.points.push_back(p);

        marker_pub.publish(marker);
        rate.sleep();
    }

    return 0;
}
