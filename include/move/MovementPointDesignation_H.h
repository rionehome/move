//
// Created by migly on 19/07/14.
//
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>

#ifndef MOVEMENTPOINTDESIGNATION_H_H
#define MOVEMENTPOINTDESIGNATION_H_H

class MovementPointDesignation
{
private:
    ros::NodeHandle n;
    ros::Subscriber odom;
    ros::Subscriber point;
    ros::Subscriber move_signal;
    ros::Publisher amount;
    ros::Publisher point_signal;

    int amount_signal_flag = 0;
    int point_signal_flag = 0;

    typedef struct
    {
        double x;
        double y;
        double z;
        double angular_x;
        double angular_y;
        double angular_z;
        double angular_w;
    } Odometry;

    Odometry odom_data{};

    void odometry(const nav_msgs::Odometry::ConstPtr &msgs);
    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void pub_msg(double distance, double angle);
    void signal(const std_msgs::Int32::ConstPtr &msg)
    {
        amount_signal_flag = msg->data;
    }

public:
    MovementPointDesignation();
    ~MovementPointDesignation();

    void calc(const std_msgs::Float64MultiArray::ConstPtr &msg);
    double calcAngle_point(double x, double y);
    static void pubSignal(const ros::Publisher &pub, int s);
    void updata()
    { ros::spinOnce(); }
    double angle_to_quaternion(double w, double z)
    {
        return std::abs((z > 0 ? 1 : 360) - this->angle_to_rad(acos(w) * 2));
    }
    double angle_to_rad(double rad)
    { return rad * 180 / M_PI; }

};


#endif //MOVEMENTPOINTDESIGNATION_H_H
