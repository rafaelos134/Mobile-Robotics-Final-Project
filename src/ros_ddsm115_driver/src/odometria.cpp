#include "odometria.hpp"
#include <cmath> 
#include <geometry_msgs/PoseStamped.h> 


OdometriaTF::OdometriaTF() {
    // valores iniciais
    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
    v_ = 0.0;
    w_ = 0.0;
    last_time_ = ros::Time::now();

    
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 50); // publica a odometria
    path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 10); // publica o caminho percorrido
    
    
    path_msg_.header.frame_id = "odom"; 
    cmd_sub_ = nh_.subscribe("/cmd_vel", 10, &OdometriaTF::cmdCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.05), &OdometriaTF::update, this);
}

void OdometriaTF::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    v_ = msg->linear.x;
    w_ = msg->angular.z;
}

void OdometriaTF::update(const ros::TimerEvent& event) {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();
    last_time_ = current_time;

    // calculo da posicao e angulo do robo para a odometria de roda
    double delta_x = (v_ * cos(th_)) * dt;
    double delta_y = (v_ * sin(th_)) * dt;
    double delta_th = w_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // Converte angulo para Quaternion
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

    // TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";       
    odom_trans.child_frame_id = "base_dummy";   
    
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster_.sendTransform(odom_trans);

    // publica a odometria
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_dummy";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = v_;
    odom.twist.twist.angular.z = w_;

    odom_pub_.publish(odom);
    
    // Cria a pose atual
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = current_time;
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose.position.x = x_;
    pose_stamped.pose.position.y = y_;
    pose_stamped.pose.orientation = odom_quat;

    // cria historico
    path_msg_.header.stamp = current_time;
    path_msg_.poses.push_back(pose_stamped);

    // Publica
    path_pub_.publish(path_msg_);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometria_node");
    OdometriaTF node;
    ros::spin();
    return 0;
}