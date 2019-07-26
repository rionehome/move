//
// Created by migly-home on 19/07/26.
//

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "../include/move/velocity.h"
#include "../include/move/MovementVelocityDesignation_H.h"

Velocity::Velocity(ros::NodeHandle *n)
{
    printf("Start class of 'Velocity'\n");
    this->odom = n->subscribe("/odom", 1000, &Velocity::setOdometry, this);
    this->velocity = n->subscribe("/move/velocity", 1000, &Velocity::callbackVelocity, this);
    this->twist = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
}
Velocity::~Velocity()
{
    printf("Shutdown class of 'Velocity'\n");
}

void Velocity::setOdometry(const nav_msgs::Odometry::ConstPtr &msg)
{
    this->odometry.x = msg->pose.pose.position.x;
    this->odometry.y = msg->pose.pose.position.y;
    this->odometry.z = msg->pose.pose.position.z;
    this->odometry.angular_x = msg->pose.pose.orientation.x;
    this->odometry.angular_y = msg->pose.pose.orientation.y;
    this->odometry.angular_z = msg->pose.pose.orientation.z;
    this->odometry.angular_w = msg->pose.pose.orientation.w;
    this->odometry.linear_speed = msg->twist.twist.linear.x;
    this->odometry.angular_speed = msg->twist.twist.angular.z;
    this->odometry.enable = true;
}

void Velocity::callbackVelocity(const nav_msgs::Odometry::ConstPtr &msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Velocity");
    ros::Rate loop_rate(10);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
