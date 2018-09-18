#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include "T_Move.hpp"

T_Move tmove;

class MovementVelocityDesignation {
public:
	MovementVelocityDesignation();
	~MovementVelocityDesignation();

	double targetV = 0;
	double targetA = 0;
	double targetV_a = 0;
	double targetA_a = 0;

	ros::NodeHandle n;
	ros::Subscriber odom;
	ros::Subscriber amountmove;
	ros::Publisher move;
	ros::Publisher signal;

	void setOdom(const nav_msgs::Odometry::ConstPtr &odom) {tmove.setOdometry(odom);}
	void calc(const std_msgs::Float64MultiArray::ConstPtr &msgs);

};

MovementVelocityDesignation::MovementVelocityDesignation() {

	printf("Start class of 'MovementVelocityDesignation'\n");

	this->odom = n.subscribe("/odom", 1000, &MovementVelocityDesignation::setOdom, this);
	this->amountmove = n.subscribe("/move/velocity", 1000, &MovementVelocityDesignation::calc, this);
	this->move = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	this->signal = n.advertise<std_msgs::Int32 >("/move/signal", 1000);

}

MovementVelocityDesignation::~MovementVelocityDesignation() {

	printf("Shutdown class of 'MovementVelocityDesignation'\n");

}

void MovementVelocityDesignation::calc(const std_msgs::Float64MultiArray::ConstPtr &msgs) {

	//[0]: 速度, [1]: 加速度, [2]: 角速度, [3]: 角加速度
	this->targetV = msgs->data[0];
	this->targetV_a = msgs->data[1];
	this->targetA = msgs->data[2];
	this->targetA_a = msgs->data[3];
	printf("debug\n");

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementVelocity");

	MovementVelocityDesignation amount;

	ros::Rate loop_rate(8);

	while (ros::ok()) {

		tmove.update();

		tmove.pubTwist(amount.move, tmove.functionStraight(amount.targetV_a, amount.targetV), tmove.functionTurn(amount.targetA_a, amount.targetA));

		loop_rate.sleep();

	}


	return 0;
}