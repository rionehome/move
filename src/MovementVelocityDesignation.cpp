#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include "T_Move.hpp"
#include <kobuki_msgs/WheelDropEvent.h>

T_Move tmove;

class MovementVelocityDesignation {
public:
	MovementVelocityDesignation();
	~MovementVelocityDesignation();

	double targetV = 0;
	double targetA = 0;
	double targetV_a = 0;
	double targetA_a = 0;
	bool status = true;

	ros::NodeHandle n;
	ros::Subscriber odom;
	ros::Subscriber amountmove;
	ros::Subscriber wheel_drop;
	ros::Publisher move;
	ros::Publisher signal;

	void setOdom(const nav_msgs::Odometry::ConstPtr &odom) {tmove.setOdometry(odom);}
	void calc(const std_msgs::Float64MultiArray::ConstPtr &msgs);
	void wheel_drop_callback(const kobuki_msgs::WheelDropEvent::ConstPtr &msgs) {
		printf("%d\n", (int)msgs->state );
		status = (int)msgs->state == 1 ? false : true;
	}
};

MovementVelocityDesignation::MovementVelocityDesignation() {

	printf("Start class of 'MovementVelocityDesignation'\n");

	this->odom = n.subscribe("/odom", 1000, &MovementVelocityDesignation::setOdom, this);
	this->amountmove = n.subscribe("/move/velocity", 1000, &MovementVelocityDesignation::calc, this);
	this->move = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	this->signal = n.advertise<std_msgs::Int32 >("/move/signal", 1000);
	this->wheel_drop = n.subscribe("/mobile_base/events/wheel_drop", 1000, &MovementVelocityDesignation::wheel_drop_callback, this);
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
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementVelocity");

	MovementVelocityDesignation velocity;

	tmove.init();

	ros::Rate loop_rate(8);

	tmove.resetAmount();

	while (ros::ok() && velocity.status) {

		tmove.update();

		tmove.pubTwist(velocity.move, tmove.calcVelocityStraight(velocity.targetV_a, velocity.targetV), tmove.calcVelocityTurn(velocity.targetA_a, velocity.targetA));

		loop_rate.sleep();
	}
	return 0;
}