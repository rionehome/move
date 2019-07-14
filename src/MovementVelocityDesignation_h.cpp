#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include "T_Move.hpp"
#include "../include/move/MovementVelocityDesignation_H.h"

MovementVelocityDesignation::MovementVelocityDesignation()
{
    printf("Start class of 'MovementVelocityDesignation'\n");
    this->odom = n.subscribe("/odom", 1000, &MovementVelocityDesignation::setOdom, this);
    this->amount_move = n.subscribe("/move/velocity", 1000, &MovementVelocityDesignation::calc, this);
    this->move = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
}

MovementVelocityDesignation::~MovementVelocityDesignation()
{
    printf("Shutdown class of 'MovementVelocityDesignation'\n");
}

void MovementVelocityDesignation::calc(const std_msgs::Float64MultiArray::ConstPtr &msgs)
{

    //[0]: 速度, [1]: 加速度, [2]: 角速度, [3]: 角加速度
    this->targetV = msgs->data[0];
    this->targetV_a = msgs->data[1];
    this->targetA = msgs->data[2];
    this->targetA_a = msgs->data[3];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MovementVelocity");
    MovementVelocityDesignation amount;
    t_move.init();
    ros::Rate loop_rate(8);
    t_move.resetAmount();
    while (ros::ok()) {
        t_move.update();
        t_move.pubTwist(amount.move,
                        t_move.calcVelocityStraight(amount.targetV_a, amount.targetV),
                        t_move.calcVelocityTurn(amount.targetA_a, amount.targetA));
        loop_rate.sleep();
    }
    return 0;
}