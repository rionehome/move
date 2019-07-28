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
#define Hz 100

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
    if (std::abs(msg->linear_rate) <= 1.0)
        this->last_linear = msg->linear_rate * MAX_LINEAR;
    if (std::abs(msg->angular_rate) <= 1.0)
        this->last_angular = msg->angular_rate * MAX_ANGULAR;
}

void Velocity::velocity_update()
{
    if (this->stack_linear != this->last_linear) {
        this->stack_linear +=
            this->last_linear_acceleration * (std::signbit(this->last_linear - this->stack_linear) ? -1 : 1);

    }
    if (std::abs(this->stack_linear) >= std::abs(this->last_linear)) {
        //制限設定
        this->stack_linear = this->last_linear;
    }
    else {
        //速度更新
        this->stack_linear += this->last_linear_acceleration * (std::signbit(this->last_linear) ? -1 : 1);
    }
    if (std::abs(this->stack_angular) >= std::abs(this->last_angular)) {
        //制限設定
        this->stack_angular = this->last_angular;
    }
    else {
        //速度更新
        this->stack_angular += this->last_angular_acceleration * (std::signbit(this->last_angular) ? -1 : 1);
    }

    //停止
    if (this->last_linear_acceleration == 0.0 && this->last_linear == 0.0) this->stack_linear = 0.0;
    if (this->last_angular_acceleration == 0.0 && this->last_angular == 0.0) this->stack_angular = 0.0;

    if (!(this->stack_linear == 0.0 && this->stack_angular == 0.0))
        this->publishTwist(this->stack_linear, this->stack_angular);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity");
    ros::NodeHandle n;
    ros::Rate loop_rate(Hz);
    Velocity velocity = Velocity(&n);
    ros::WallTime before;
    while (ros::ok()) {
        ros::spinOnce();
        velocity.velocity_update();
        before = ros::WallTime::now();
        loop_rate.sleep();
        std::cout << ros::WallTime::now() - before << '\n';
        printf("#######################\n");
    }
    return 0;
}
