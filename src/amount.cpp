//
// Created by migly-home on 19/07/26.
//
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "move/Amount.h"
#include <nav_msgs/Odometry.h>
#include <move/Velocity.h>
#include "../include/move/amount.h"


#define Hz 10

Amount::Amount(ros::NodeHandle *n)
{
    printf("Start class of 'Amount'\n");
    this->amount_sub = n->subscribe("/move/amount", 1000, &Amount::callbackAmount, this);
    this->odometry_sub = n->subscribe("/odom", 1000, &Amount::callbackOdometry, this);
    this->twist_pub = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    this->velocity_pub = n->advertise<move::Velocity>("/move/velocity", 1000);
}

Amount::~Amount()
{
    printf("Shutdown class of 'Amount'\n");
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
    this->diff_angular[1] = this->target_angle - this->sensor_angle;
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

    double result_linear, result_angular;
    move::Velocity velocity_data;

    std::cout << this->target_distance - this->sensor_distance << '\n';
    if (std::abs(this->diff_linear[1] - this->diff_linear[0]) < 0.005
        && std::abs(this->target_distance - this->sensor_distance) < 0.01) {
        this->move_flag = false;
        velocity_data.linear_rate = 0;
        velocity_data.angular_rate = 0;
        this->velocity_pub.publish(velocity_data);
        printf("finish\n");
        return;
    }
    result_linear = this->distancePidControl(1.5, 0.01, 0.0045);
    if (std::abs(result_linear) > MAX_LINEAR)
        result_linear = MAX_LINEAR * (std::signbit(result_linear) ? -1 : 1);

    result_angular = this->angularPidControl(1, 0, 0);
    if (std::abs(result_angular) > MAX_ANGULAR)
        result_angular = MAX_ANGULAR * (std::signbit(result_angular) ? -1 : 1);

    velocity_data.linear_rate = result_linear;
    velocity_data.angular_rate = 0;
    this->velocity_pub.publish(velocity_data);
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
