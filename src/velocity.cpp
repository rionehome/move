//
// Created by migly-home on 19/07/26.
//
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "move/Velocity.h"
#include <nav_msgs/Odometry.h>
#include "../include/move/velocity.h"


#define Hz 10

Velocity::Velocity(ros::NodeHandle *n)
{
    printf("Start class of 'Velocity'\n");
    this->velocity_sub = n->subscribe("/move/velocity", 1000, &Velocity::callbackVelocity, this);
    this->odometry_sub = n->subscribe("/odom", 1000, &Velocity::callbackOdometry, this);
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
    //if (std::abs(angular_z) > MAX_ANGULAR) angular_z = MAX_ANGULAR * (std::signbit(angular_z) ? -1 : 1);
    twist.angular.z = angular_z;
    this->twist_pub.publish(twist);
}

void Velocity::linearPidControl(double Kp, double Ki, double Kd)
{
    /*
     * Linear PID制御
     */

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
    std::cout << p + i + d << '\n';
    return p + i + d;
}

void Velocity::velocity_update()
{
    double output_linear, output_angular;

    this->stack_angular += this->angularPidControl(0.3, 0, 0);
    output_linear = 0;

    //停止
    //if (this->input_linear == 0.0) output_linear = 0.0;
    //if (this->target_angular == 0.0) output_angular = 0.0;

    this->publishTwist(output_linear, this->stack_angular);
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
    }
    return 0;
}
