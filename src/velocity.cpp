//
// Created by migly-home on 19/07/26.
//
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include "move/Velocity.h"
#include <nav_msgs/Odometry.h>
#include "../include/move/velocity.h"


#define Hz 10

Velocity::Velocity(ros::NodeHandle *n)
{
    printf("Start class of 'Velocity'\n");
    this->velocity_sub = n->subscribe("/move/velocity", 1000, &Velocity::callbackVelocity, this);
    this->odometry_sub = n->subscribe("/odom", 1000, &Velocity::callbackOdometry, this);
    this->wheel_drop_sub = n->subscribe("/mobile_base/events/wheel_drop", 1000, &Velocity::callbackWheeDrop, this);
    this->twist_pub = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
}

Velocity::~Velocity()
{
    printf("Shutdown class of 'Velocity'\n");
}

void Velocity::publishTwist(double liner_x, double angular_z)
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

double Velocity::linearPidControl(double Kp, double Ki, double Kd)
{
    /*
     * Linear PID制御
     */
    double p, i, d;
    this->diff_linear[0] = this->diff_linear[1];
    this->diff_linear[1] = target_linear - sensor_linear;
    this->integral_linear += (this->diff_linear[1] + this->diff_linear[0]) / 2.0 * (1.0 / Hz);
    p = Kp * this->diff_linear[1];
    i = Ki * this->integral_linear;
    d = Kd * (this->diff_linear[1] - this->diff_linear[0]) / (1.0 / Hz);
    return p + i + d;
}

double Velocity::angularPidControl(double Kp, double Ki, double Kd)
{
    /*
     * angular PID制御
     */
    double p, i, d;
    this->diff_angular[0] = this->diff_angular[1];
    this->diff_angular[1] = target_angular - sensor_angular;
    this->integral_angular += (this->diff_angular[1] + this->diff_angular[0]) / 2.0 * (1.0 / Hz);
    p = Kp * this->diff_angular[1];
    i = Ki * this->integral_angular;
    d = Kd * (this->diff_angular[1] - this->diff_angular[0]) / (1.0 / Hz);
    return p + i + d;
}

void Velocity::velocity_update()
{
    if (!this->move_flag)
        return;

    this->stack_linear += this->linearPidControl(0.1, 0, 0);
    this->stack_angular += this->angularPidControl(0.7, 0.05, 0);

    //停止
    if (std::abs(this->stack_linear) < 0.02 && this->target_linear == 0.0) this->stack_linear = 0.0;
    if (std::abs(this->stack_angular) < 0.05 && this->target_angular == 0.0) this->stack_angular = 0.0;
    if (this->stack_linear == 0.0 && this->stack_angular == 0.0 &&
        this->target_linear == 0.0 && this->target_angular == 0.0)
        this->move_flag = false;
    //規定値オーバー
    if (std::abs(this->stack_linear) > MAX_LINEAR)
        this->stack_linear = MAX_LINEAR * (std::signbit(this->stack_linear) ? -1 : 1);
    if (std::abs(this->stack_angular) > MAX_ANGULAR)
        this->stack_angular = MAX_ANGULAR * (std::signbit(this->stack_angular) ? -1 : 1);

    printf("linear:%f, angular%f\n", this->stack_linear, this->stack_angular);
    this->publishTwist(this->stack_linear, this->stack_angular);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity");
    ros::NodeHandle n;
    ros::Rate loop_rate(Hz);
    Velocity velocity = Velocity(&n);
    while (ros::ok()) {
        ros::spinOnce();
        velocity.velocity_update();
        loop_rate.sleep();
    }
    return 0;
}
