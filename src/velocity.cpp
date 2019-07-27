//
// Created by migly-home on 19/07/26.
//
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "move/Velocity.h"
#include <nav_msgs/Odometry.h>
#include "../include/move/velocity.h"

#define MAX_LINEAR 0.7 // m/s
#define MAX_ANGULAR 1.9 // rad

Velocity::Velocity(ros::NodeHandle *n)
{
    printf("Start class of 'Velocity'\n");
    this->velocity_sub = n->subscribe("/move/velocity", 1000, &Velocity::callbackVelocity, this);
    this->twist_pub = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
}

Velocity::~Velocity()
{
    printf("Shutdown class of 'Velocity'\n");
}

void Velocity::publishTwist(double liner_x, double angular_z)
{
    geometry_msgs::Twist twist = geometry_msgs::Twist();
    twist.linear.x = liner_x;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = angular_z;
    this->twist_pub.publish(twist);
}

void Velocity::callbackVelocity(const move::Velocity::ConstPtr &msg)
{
    /*
     * 直進、回転の加速度と速度の情報を受け取り.
     * linear:0.7m/sが最大.
     * angular:110deg/sが最大.
     */
    if (std::abs(msg->linear_acceleration_rate) <= 1.0)
        this->last_linear_acceleration = msg->linear_acceleration_rate * MAX_LINEAR;
    if (std::abs(msg->linear_rate) <= 1.0)
        this->last_linear = msg->linear_rate * MAX_LINEAR;
    if (std::abs(msg->angular_acceleration_rate) <= 1.0)
        this->last_angular_acceleration = msg->angular_acceleration_rate * MAX_ANGULAR;
    if (std::abs(msg->angular_rate) <= 1.0)
        this->last_angular = msg->angular_rate * MAX_ANGULAR;
}

void Velocity::velocity_update()
{
    //速度更新
    this->stack_linear += this->last_linear_acceleration;
    this->stack_angular += this->last_angular_acceleration;

    //制限設定
    if (std::abs(this->stack_linear) > std::abs(this->last_linear)) this->stack_linear = this->last_linear;
    if (std::abs(this->stack_angular) > std::abs(this->last_angular)) this->stack_angular = this->last_angular;

    //停止
    if (this->last_linear_acceleration == 0.0 && this->last_linear == 0.0) this->stack_linear = 0.0;
    if (this->last_angular_acceleration == 0.0 && this->last_angular == 0.0) this->stack_angular = 0.0;

    this->publishTwist(this->stack_linear, this->stack_angular);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Velocity");
    ros::Rate loop_rate(10);
    ros::NodeHandle n;
    Velocity velocity = Velocity(&n);
    while (ros::ok()) {
        ros::spinOnce();
        velocity.velocity_update();
        loop_rate.sleep();
    }
    return 0;
}
