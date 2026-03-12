#ifndef LEITOR_TELEOP_NODE_HPP
#define LEITOR_TELEOP_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "cinematica.hpp"

class LeitorTeleopNode {
public:
    LeitorTeleopNode(ros::NodeHandle& nh);

private:
    void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg);

    ros::Subscriber sub_;
    ros::Publisher pub_front_left_;
    ros::Publisher pub_front_right_;
    ros::Publisher pub_back_left_;
    ros::Publisher pub_back_right_;
};

#endif