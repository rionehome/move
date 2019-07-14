//
// Created by migly on 19/07/13.
//

#ifndef MOVEMENTAMOUNTDESIGNATION_H_H
#define MOVEMENTAMOUNTDESIGNATION_H_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include "../../src/T_Move.hpp"

using namespace std;

T_Move t_move;

class MovementAmountDesignation
{
private:
    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void setOdom(const nav_msgs::Odometry::ConstPtr &msg)
    { t_move.setOdometry(msg); }

public:
    MovementAmountDesignation();
    ~MovementAmountDesignation();

    ros::NodeHandle n;
    ros::Subscriber amount;
    ros::Subscriber odom_sub;
    ros::Publisher velocity;
    ros::Publisher pub_signal;
    ros::Publisher twist;

    double call_liner[2];
    double call_angle[2];
    bool move_signal = false;
    bool move_status = false;
    bool core_status = false;
};

#endif //MOVEMENTAMOUNTDESIGNATION_H_H
