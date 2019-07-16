//
// Created by migly on 19/07/14.
//

#ifndef MOVEMENTVELOCITYDESIGNATION_H
#define MOVEMENTVELOCITYDESIGNATION_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include "../../src/T_Move.hpp"

T_Move t_move;

class MovementVelocityDesignation
{
public:
    MovementVelocityDesignation();
    ~MovementVelocityDesignation();

    double targetV = 0;
    double targetA = 0;
    double targetV_a = 0;
    double targetA_a = 0;

    ros::NodeHandle n;
    ros::Subscriber odom;
    ros::Subscriber amount_move;
    ros::Publisher move;

    void setOdom(const nav_msgs::Odometry::ConstPtr &msgs)
    { t_move.setOdometry(msgs); }
    void calc(const std_msgs::Float64MultiArray::ConstPtr &msgs);
};

#endif //MOVEMENTVELOCITYDESIGNATION_H
