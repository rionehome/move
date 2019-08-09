#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math

import numpy
import rospy
from nav_msgs.msg import Odometry
from move.msg import Amount
import matplotlib.pyplot as plt
import numpy as np


class Visualizer:
    def __init__(self):
        self.distance_stack = np.asarray([])
        self.angle_stack = np.asarray([])
        self.target_angle = 0
        self.target_distance = 0
        self.init_x = 0
        self.init_y = 0
        self.sensor_x = 0
        self.sensor_y = 0
        self.sensor_q_w = 0
        self.sensor_q_z = 0
        self.sensor_angle = 0
        self.sensor_distance = 0
        self.last_q_w = 0
        self.last_q_z = 0
        
        rospy.init_node("move_visualizer_amount")
        rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber("/move/amount", Amount, self.amount_callback)
    
    @staticmethod
    def to_angle(rad):
        return rad * 180 / math.pi
    
    @staticmethod
    def to_radian(angle):
        return (angle * math.pi) / 180
    
    def to_quaternion_ang(self, w, z):
        if abs(z > 0):
            return 1 - self.to_angle(math.acos(w) * 2)
        else:
            return 360 - self.to_angle(math.acos(w) * 2)
    
    def history_distance(self, x, y):
        return math.sqrt((x - self.init_x) ** 2 + (y - self.init_y) ** 2) * numpy.sign(x - self.init_x)
    
    def history_angle(self, angle):
        __delta_angle__ = angle - self.to_quaternion_ang(self.last_q_w, self.last_q_z)
        self.last_q_w = self.sensor_q_w
        self.last_q_z = self.sensor_q_z
        return __delta_angle__
    
    def amount_callback(self, msg):
        # type: (Amount) -> None
        self.sensor_angle = 0
        self.target_distance = msg.distance
        self.target_angle = msg.angle
        self.init_x = self.sensor_x
        self.init_y = self.sensor_y
    
    def odometry_callback(self, msg):
        # type: (Odometry) -> None
        plt.cla()
        self.sensor_x = msg.pose.pose.position.x
        self.sensor_y = msg.pose.pose.position.y
        self.sensor_q_w = msg.pose.pose.orientation.w
        self.sensor_q_z = msg.pose.pose.orientation.z
        self.sensor_distance = self.history_distance(self.sensor_x, self.sensor_y)
        self.sensor_angle += self.history_angle(self.to_quaternion_ang(self.sensor_q_w, self.sensor_q_z))
        
        if len(self.distance_stack) >= 100:
            self.distance_stack = np.delete(self.distance_stack, 0)
        self.distance_stack = np.append(self.distance_stack, self.sensor_distance)
        
        if len(self.angle_stack) >= 100:
            self.angle_stack = np.delete(self.angle_stack, 0)
        self.angle_stack = np.append(self.angle_stack, self.to_radian(self.sensor_angle))
        
        t = np.arange(0, len(self.distance_stack), 1)
        plt.plot(t, self.distance_stack)
        plt.plot(t, self.angle_stack)
        plt.xlim(0, 100)
        plt.ylim(-3, 3)
        plt.hlines([self.target_distance], 0, 100, "blue", linestyles='dashed')
        plt.hlines([self.to_radian(self.target_angle)], 0, 100, "red", linestyles='dashed')
        plt.pause(.01)


if __name__ == '__main__':
    Visualizer()
    rospy.spin()
