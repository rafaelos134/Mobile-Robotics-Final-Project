#include "leitor_teleop_node.hpp"
#include "cinematica.hpp"
#include <cmath>


LeitorTeleopNode::LeitorTeleopNode(ros::NodeHandle& nh) {
  
    
    // Se inscreve no cmd_vel (Vem do Teclado OU do Campos.py)
    sub_ = nh.subscribe("/cmd_vel", 10, &LeitorTeleopNode::teleopCallback, this);

    // Publica nas rodas da simulacao
    pub_front_left_  = nh.advertise<std_msgs::Float64>("/front_left_wheel/target_velocity", 1);
    pub_front_right_ = nh.advertise<std_msgs::Float64>("/front_right_wheel/target_velocity", 1);
    pub_back_left_   = nh.advertise<std_msgs::Float64>("/rear_left_wheel/target_velocity", 1);
    pub_back_right_  = nh.advertise<std_msgs::Float64>("/rear_right_wheel/target_velocity", 1);

    ROS_INFO("Driver de Rodas Iniciado");
}

// Callback
void LeitorTeleopNode::teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    
    double v = msg->linear.x;
    double w = msg->angular.z;


    WheelVelocities vel = calculate_kinematics(v, w);

    std_msgs::Float64 left_msg, right_msg;
    left_msg.data = vel.left_rad_s;
    right_msg.data = vel.right_rad_s;

    // Publica
    pub_front_left_.publish(left_msg);
    pub_back_left_.publish(left_msg); 

    pub_front_right_.publish(right_msg);
    pub_back_right_.publish(right_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "leitor_teleop_node");
    ros::NodeHandle nh;
    LeitorTeleopNode leitor(nh);
    ros::spin();
    return 0;
}