#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time
import math

def get_yaw_from_quaternion(q):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)

def transform_laser_to_global(laser_data, robot_pos, robot_ori):
    x_r, y_r = robot_pos
    theta_r = robot_ori[2]
    global_points = []
    

    if not np.isfinite(x_r) or not np.isfinite(y_r):
        return []

    for ang, dist in laser_data:
        if np.isfinite(dist) and 0.05 < dist < 5.0:
            x_local = dist * np.cos(ang)
            y_local = dist * np.sin(ang)
            x_global = x_r + x_local*np.cos(theta_r) - y_local*np.sin(theta_r)
            y_global = y_r + x_local*np.sin(theta_r) + y_local*np.cos(theta_r)
            global_points.append([x_global, y_global])
    return global_points

def att_force(q, goal, katt=1.0):
    return katt * (goal - q)

def rep_force(q, laser_global, R=0.30, krep=1.0):
    if len(laser_global) == 0: return np.zeros(2)
    
    laser_global = np.array(laser_global)
    v = q - laser_global
    d = np.linalg.norm(v, axis=1)
    
    if np.any(np.isnan(d)):
        return np.zeros(2)

    d = np.maximum(d, 1e-6).reshape(-1, 1)
    
    # Cálculo da força
    rep = (1.0/d**2) * ((1.0/d) - (1.0/R)) * (v/d)
    
    rep[d.flatten() > R] = 0.0
    
    return krep * np.sum(rep, axis=0)






class SkidSteerPotentialFieldNode:

    def __init__(self):
        rospy.init_node("skid_potential_field_node")

        self.L = 0.22; 
        self.active = True
        self.goal = np.array([6.0, 0.0])
        self.GOAL_TOLERANCE = 0.20 

        # Ganhos
        self.REP_R = 0.5     
        self.REP_GAIN = 0.4
        self.ATT_GAIN = 7
        self.LINEAR_GAIN = 1.0
        self.ANGULAR_GAIN = 1.0

        # Limites
        self.MAX_V = 0.4      
        self.MAX_W = 0.4      
        self.MAX_TIME = 120   

        self.robot_pos = np.array([0.0, 0.0])
        self.robot_ori = np.array([0.0, 0.0, 0.0]) 
        self.last_scan = None
        self.start_time = time.time()
        self.goal_reached = False

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/enable_auto", Bool, self.enable_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        rospy.loginfo("Campos Potenciais: Publicando em /cmd_vel. Aguardando dados...")

    def enable_callback(self, msg: Bool):
        if msg.data and not self.active:
            rospy.loginfo("ATIVANDO CONTROLE AUTOMÁTICO")
            self.start_time = time.time()
        elif not msg.data and self.active:
            rospy.loginfo("DESATIVANDO CONTROLE AUTOMÁTICO")
            self.stop_robot()
        self.active = msg.data

    def odom_callback(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        
        if np.isnan(px) or np.isnan(py):
            return

        self.robot_pos[0] = px
        self.robot_pos[1] = py
        orientation_q = msg.pose.pose.orientation
        self.robot_ori[2] = get_yaw_from_quaternion(orientation_q)

    def scan_callback(self, msg: LaserScan):
        if not self.active: return 
        if self.goal_reached:
            self.stop_robot()
            return
        if time.time() - self.start_time > self.MAX_TIME:
            self.stop_robot()
            return

        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        self.last_scan = list(zip(angles, msg.ranges))
        self.compute_control()

    def compute_control(self):
        if self.last_scan is None: return

        # Verifica Objetivo
        dist_to_goal = np.linalg.norm(self.goal - self.robot_pos)
        if dist_to_goal < self.GOAL_TOLERANCE:
            rospy.loginfo("CHEGOU!")
            self.goal_reached = True
            self.stop_robot()
            self.active = False
            return 

        
        laser_global = transform_laser_to_global(self.last_scan, self.robot_pos, self.robot_ori)
        f_att = att_force(self.robot_pos, self.goal, self.ATT_GAIN)
        f_rep = rep_force(self.robot_pos, laser_global, R=self.REP_R, krep=self.REP_GAIN)
        
        if np.any(np.isnan(f_rep)):
            f_rep = np.zeros(2)

        f_total = f_att + f_rep


        xd, yd = f_total
        theta = self.robot_ori[2]
        d = self.L / 2.0  

        J = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta) / d, np.cos(theta) / d]
        ])

        v_cmd, w_cmd = J @ np.array([xd, yd])

        v_cmd *= self.LINEAR_GAIN
        w_cmd *= self.ANGULAR_GAIN

        theta_d = np.arctan2(yd, xd)
        erro_theta = np.arctan2(np.sin(theta_d - theta), np.cos(theta_d - theta))

        if abs(erro_theta) > np.deg2rad(45):
            v_cmd *= 0.25

        v_cmd = np.clip(v_cmd, -self.MAX_V, self.MAX_V)
        w_cmd = np.clip(w_cmd, -self.MAX_W, self.MAX_W)


        # publica velocidades
        twist_msg = Twist()
        twist_msg.linear.x = v_cmd
        twist_msg.angular.z = w_cmd 
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)

if __name__ == "__main__":
    try:
        node = SkidSteerPotentialFieldNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass
