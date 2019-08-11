//
// Created by migly-home on 19/07/26.
//
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "move/Amount.h"
#include <nav_msgs/Odometry.h>
#include <move/Velocity.h>
#include <move/AmountAction.h>
#include <actionlib/server/simple_action_server.h>
#include "../include/move/amount.h"


#define Hz 10

Amount::Amount(ros::NodeHandle *n)
{
    printf("Start class of 'Amount'\n");
    //this->amount_sub = n->subscribe("/move/amount", 1000, &Amount::callbackAmount, this);
    this->odometry_sub = n->subscribe("/odom", 1000, &Amount::callbackOdometry, this);
    this->twist_pub = n->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
    this->velocity_pub = n->advertise<move::Velocity>("/move/velocity", 1000);
    this->server = new Server(*n, "/move/amount", false);
    this->server->start();
}

Amount::~Amount()
{
    printf("Shutdown class of 'Amount'\n");
}

void Amount::rosUpdate()
{
    ros::spinOnce();
    //actionの更新が確認された場合
    if (this->server->isNewGoalAvailable()) {
        this->current_goal = this->server->acceptNewGoal();
        this->move_flag = true;
        this->sensor_angle = 0;
        this->target_linear_rate = this->current_goal->amount.velocity.linear_rate;
        this->target_distance = this->current_goal->amount.distance;
        this->target_angular_rate = this->current_goal->amount.velocity.angular_rate;
        this->target_angle = this->current_goal->amount.angle;
        this->initial_x = this->sensor_x;
        this->initial_y = this->sensor_y;
        this->last_q_w = this->sensor_q_w;
        this->last_q_z = this->sensor_q_z;
    }
    //actionの中断が確認された場合
    if (this->server->isActive() && this->server->isPreemptRequested()) {
        move::Velocity velocity_data;
        move_flag = false;
        velocity_data.linear_rate = 0.0;
        velocity_data.angular_rate = 0.0;
        this->velocity_pub.publish(velocity_data);
        this->server->setAborted();
    }
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

    double result_linear_rate, result_angular_rate;
    move::Velocity velocity_data;

    result_linear_rate = this->distancePidControl(1.5, 0.01, 0.0045) / MAX_LINEAR;
    if (std::abs(result_linear_rate) > 1.0)
        result_linear_rate = std::signbit(result_linear_rate) ? -1 : 1;
    if (std::abs(result_linear_rate) > this->target_linear_rate)
        result_linear_rate = this->target_linear_rate * (std::signbit(result_linear_rate) ? -1 : 1);
    if (this->target_distance == 0)
        result_linear_rate = 0;

    result_angular_rate = this->angularPidControl(0.03, 0.0001, 0.000035) / MAX_ANGULAR;
    if (std::abs(result_angular_rate) > 1.0)
        result_angular_rate = std::signbit(result_angular_rate) ? -1 : 1;
    if (std::abs(result_angular_rate) > this->target_angular_rate)
        result_angular_rate = this->target_angular_rate * (std::signbit(result_angular_rate) ? -1 : 1);
    if (this->target_angle == 0)
        result_angular_rate = 0;
    std::cout << result_angular_rate << '\n';

    //action feedback
    printf("distance:%f, angle%f\n", this->sensor_distance, this->sensor_angle);
    move::AmountFeedback feedback;
    feedback.current_amount.distance = this->sensor_distance;
    feedback.current_amount.angle = this->sensor_angle;
    this->server->publishFeedback(feedback);

    //終了判定
    bool finish_distance = false, finish_angle = false;
    if (std::abs(this->diff_linear[1] - this->diff_linear[0]) < 0.005
        && std::abs(this->target_distance - this->sensor_distance) < 0.01) {
        finish_distance = true;
        result_linear_rate = 0;
    }
    if (std::abs(this->diff_angular[1] - this->diff_angular[0]) < 0.07
        && std::abs(this->target_angle - this->sensor_angle) < 10) {
        finish_angle = true;
        result_angular_rate = 0;
    }

    //終了
    if (finish_distance && finish_angle) {
        move_flag = false;
        printf("finish\n");
        this->server->setSucceeded();
    }

    velocity_data.linear_rate = result_linear_rate;
    velocity_data.angular_rate = result_angular_rate;
    this->velocity_pub.publish(velocity_data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amount");
    ros::NodeHandle n;
    ros::Rate loop_rate(Hz);
    Amount amount = Amount(&n);
    while (ros::ok()) {
        amount.rosUpdate();
        amount.amount_update();
        loop_rate.sleep();
    }
    return 0;
}
