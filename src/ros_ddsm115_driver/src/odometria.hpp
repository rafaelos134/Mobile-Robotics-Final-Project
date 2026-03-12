#ifndef ODOMETRIA_HPP
#define ODOMETRIA_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h> 

class OdometriaTF {
private:
    ros::NodeHandle nh_;
    
    // Publishers e Subscribers
    ros::Publisher odom_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber cmd_sub_;
    ros::Timer timer_;
    
    tf::TransformBroadcaster odom_broadcaster_;


    double x_;
    double y_;
    double th_;
    double v_;
    double w_;

    // Armazena o caminho
    nav_msgs::Path path_msg_;

    // Controle temp
    ros::Time last_time_;

public:
    OdometriaTF(); 
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void update(const ros::TimerEvent& event);
};

#endif