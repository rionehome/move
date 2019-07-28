#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Odometry
from move.msg import Velocity
import matplotlib.pyplot as plt
import numpy as np


class Visualizer:
    def __init__(self):
        self.linear_stack = np.asarray([])
        self.angular_stack = np.asarray([])
        self.target_angular = 0

        rospy.init_node("move_visualizer")
        rospy.Subscriber("/odom", Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber("/move/velocity", Velocity, self.velocity_callback)

    def velocity_callback(self, msg):
        # type: (Velocity) -> None
        self.target_angular = msg.angular_rate * 1.9

    def odometry_callback(self, msg):
        # type: (Odometry) -> None
        plt.cla()
        if len(self.linear_stack) >= 100:
            self.linear_stack = np.delete(self.linear_stack, 0)
        self.linear_stack = np.append(self.linear_stack, msg.twist.twist.linear.x)

        if len(self.angular_stack) >= 100:
            self.angular_stack = np.delete(self.angular_stack, 0)
        self.angular_stack = np.append(self.angular_stack, msg.twist.twist.angular.z)

        t = np.arange(0, len(self.angular_stack), 1)
        plt.plot(t, self.angular_stack)
        plt.xlim(0, 100)
        plt.ylim(-3.0, 3.0)
        plt.hlines([self.target_angular], 0, 100, "blue", linestyles='dashed')  # hlines
        plt.pause(.01)


if __name__ == '__main__':
    Visualizer()
    rospy.spin()
