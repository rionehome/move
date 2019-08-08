//
// Created by migly-home on 19/07/26.
//
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include "move/Amount.h"
#include <nav_msgs/Odometry.h>
#include "../include/move/amount.h"


#define Hz 10

Amount::Amount(ros::NodeHandle *n)
{
    printf("Start class of 'Amount'\n");
    this->amount_sub = n->subscribe("/move/amount", 1000, &Amount::callbackAmount, this);
    this->odometry_sub = n->subscribe("/odom", 1000, &Amount::callbackOdometry, this);
    this->wheel_drop_sub = n->subscribe("/mobile_base/events/wheel_drop", 1000, &Amount::callbackWheeDrop, this);
    this->twist_pub = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
}

Amount::~Amount()
{
    printf("Shutdown class of 'Amount'\n");
}

void Amount::publishTwist(double liner_x, double angular_z)
{
    geometry_msgs::Twist twist = geometry_msgs::Twist();
    if (std::abs(liner_x) > MAX_LINEAR) liner_x = MAX_LINEAR * (std::signbit(liner_x) ? -1 : 1);
    twist.linear.x = liner_x;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    if (std::abs(angular_z) > MAX_ANGULAR) angular_z = MAX_ANGULAR * (std::signbit(angular_z) ? -1 : 1);
    twist.angular.z = angular_z;
    this->twist_pub.publish(twist);
}

double Amount::distancePidControl(double Kp, double Ki, double Kd)
{
    /*
     * Linear PID制御
     */
    double p, i, d;
    this->diff_linear[0] = this->diff_linear[1];
    this->diff_linear[1] = this->target_distance - this->sensor_distance;
    this->integral_linear += (this->diff_linear[1] + this->diff_linear[0]) / 2.0 * (1.0 / Hz);
    p = Kp * this->diff_linear[1];
    i = Ki * this->integral_linear;
    d = Kd * (this->diff_linear[1] - this->diff_linear[0]) / (1.0 / Hz);
    return p + i + d;
}

double Amount::angularPidControl(double Kp, double Ki, double Kd)
{
    /*
     * angular PID制御
     */
    double p, i, d;
    this->diff_angular[0] = this->diff_angular[1];
    this->diff_angular[1] = target_angle - sensor_angular;
    this->integral_angular += (this->diff_angular[1] + this->diff_angular[0]) / 2.0 * (1.0 / Hz);
    p = Kp * this->diff_angular[1];
    i = Ki * this->integral_angular;
    d = Kd * (this->diff_angular[1] - this->diff_angular[0]) / (1.0 / Hz);
    return p + i + d;
}

void Amount::amount_update()
{
    if (!this->move_flag)
        return;

    this->publishTwist(this->distancePidControl(0.4, 0.25, 0.1), 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amount");
    ros::NodeHandle n;
    ros::Rate loop_rate(Hz);
    Amount amount = Amount(&n);
    while (ros::ok()) {
        ros::spinOnce();
        amount.amount_update();
        loop_rate.sleep();
    }
    return 0;
}
