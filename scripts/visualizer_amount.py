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
        self.angular_stack = np.asarray([])
        self.target_angular = 0
        self.target_distance = 0
        self.init_x = 0
        self.init_y = 0
        self.sensor_x = 0
        self.sensor_y = 0
        self.sensor_distance = 0

        rospy.init_node("move_visualizer_amount")
        rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber("/move/amount", Amount, self.amount_callback)

    def advanced_distance(self, x, y):
        return math.sqrt((x - self.init_x) ** 2 + (y - self.init_y) ** 2) * numpy.sign(x - self.init_x)

    def amount_callback(self, msg):
        # type: (Amount) -> None
        self.target_angular = msg.angular_rate * 1.9
        self.target_distance = msg.distance
        self.init_x = self.sensor_x
        self.init_y = self.sensor_y

    def odometry_callback(self, msg):
        # type: (Odometry) -> None
        plt.cla()
        self.sensor_x = msg.pose.pose.position.x
        self.sensor_y = msg.pose.pose.position.y
        self.sensor_distance = self.advanced_distance(self.sensor_x, self.sensor_y)

        if len(self.distance_stack) >= 100:
            self.distance_stack = np.delete(self.distance_stack, 0)
        self.distance_stack = np.append(self.distance_stack, self.sensor_distance)

        t = np.arange(0, len(self.distance_stack), 1)
        plt.plot(t, self.distance_stack)
        # plt.plot(t, self.angular_stack)
        plt.xlim(0, 100)
        plt.ylim(-3, 3)
        plt.hlines([self.target_distance], 0, 100, "blue", linestyles='dashed')
        # plt.hlines([self.target_angular], 0, 100, "red", linestyles='dashed')
        plt.pause(.01)


if __name__ == '__main__':
    Visualizer()
    rospy.spin()
