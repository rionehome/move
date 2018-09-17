#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include "P_Move.hpp"

P_Move pmove;

class MovementVelocityDesignation {
public:
	MovementVelocityDesignation();
	~MovementVelocityDesignation();

	double targetV;
	double targetA;

	ros::NodeHandle n;
	ros::Subscriber odom;
	ros::Subscriber amountmove;
	ros::Publisher move;
	ros::Publisher signal;

	void setOdom(const nav_msgs::Odometry::ConstPtr &odom) {pmove.setOdometry(odom);}
	void calc(const std_msgs::Float64MultiArray::ConstPtr &msgs);

};

MovementVelocityDesignation::MovementVelocityDesignation() {

	printf("Start class of 'MovementVelocityDesignation'\n");

	this->odom = n.subscribe("/odom", 1000, &MovementVelocityDesignation::setOdom, this);
	this->amountmove = n.subscribe("/move/amount", 1000, &MovementVelocityDesignation::calc, this);
	this->move = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	this->signal = n.advertise<std_msgs::Int32 >("/move/signal", 1000);

}

MovementVelocityDesignation::~MovementVelocityDesignation() {

	printf("Shutdown class of 'MovementVelocityDesignation'\n");

}

void MovementVelocityDesignation::calc(const std_msgs::Float64MultiArray::ConstPtr &msgs) {

	//[0]: 速度 [1]: 角速度
	this->targetV = msgs->data[0];
	this->targetA = msgs->data[1];

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementVelocity");

	MovementVelocityDesignation amount;

	ros::Rate loop_rate(8);

	while (ros::ok()) {

		pmove.update();

		//pmove.pubTwist(amount.move, pmove.functionStraight(0.5, amount.targetV), pmove.functionTurn(0.5, amount.targetA));
		pmove.pubTwist(amount.move, pmove.functionStraight(0.5, amount.targetV), pmove.functionTurn(0.5, amount.targetA));

		loop_rate.sleep();

	}


	return 0;
}