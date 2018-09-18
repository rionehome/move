#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include "T_Move_old.hpp"

#define ACCELERATION 2
#define ANGLE_ACCELERATION 0.015

using namespace std;

T_Move t_move;

class MovementAmountDesignation {
private:
	ros::NodeHandle n;
	//ros::Subscriber odom;
	//ros::Subscriber amount_distance;
	//ros::Subscriber amount_angle;
	ros::Subscriber amount;
	ros::Publisher velocity;
	ros::Publisher signal;

	void callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
	void Straight(double distance, double v);
	void Turn(double angle, double v);

public:
	MovementAmountDesignation();
	~MovementAmountDesignation();

};

MovementAmountDesignation::MovementAmountDesignation() {

	printf("start class of 'MovementAmountDesignation'\n");

	//this->odom = n.subscribe("/odom", 1000, &MovementAmountDesignation::odometry, this);
	this->amount = n.subscribe("/move/amount", 1000, &MovementAmountDesignation::callback, this);
	//this->move = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	this->velocity = n.advertise<std_msgs::Float64MultiArray>("/move/velocity", 1000);
	this->signal = n.advertise<std_msgs::Int32 >("/move/signal", 1000);

}


MovementAmountDesignation::~MovementAmountDesignation() {

	printf("shutdown class of 'MovementAmountDesignation'\n");
}

void MovementAmountDesignation::callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {

	//[0]:liner距離 , [1]:liner速度 , [2]:angle距離 , [3]:angle速度

	if (msg->data[2] == 0) { //linter

		this->Straight(msg->data[0], msg->data[1]);

	} else if (msg->data[0] == 0) { //angle

		this->Turn(msg->data[2], msg->data[3]);

	} else {

		printf("error\n");

	}

}

void MovementAmountDesignation::Straight(double distance, double v) {

	if (t_move.v_data.p3 == 0) {

		ros::Rate loop_rate(8);

		t_move.createStraight(t_move.getPosition("x"), t_move.getPosition("y"), v, distance, ACCELERATION, 0.08);

		while (ros::ok()) {

			t_move.update();

			t_move.v_data.elapsed = hypot(t_move.getPosition("x") - t_move.v_data.x0, t_move.getPosition("y") - t_move.v_data.y0);

			if ( t_move.v_data.elapsed > abs(t_move.v_data.p3)) {
				t_move.pubVelocity(this->velocity, 0, 0, 0, 0);
				break;
			}

			loop_rate.sleep();

			printf("%f\n", t_move.v_data.elapsed );

			if (distance > 0) {
				t_move.pubVelocity(this->velocity, t_move.calcStraight(t_move.v_data.elapsed), 0.3, 0, 0);
				//this->publish_twist(this->move, this->calcStraight(v_data.elapsed), 0);
			} else {
				t_move.pubVelocity(this->velocity, -1 * t_move.calcStraight(t_move.v_data.elapsed), 0.3, 0, 0);
				//this->publish_twist(this->move, -1 * this->calcStraight(v_data.elapsed), 0);
			}

		}

		t_move.v_data.p3 = 0;
		t_move.pubSignal(this->signal, 1);

	} else {

		if (t_move.sign(t_move.v_data.distance) == t_move.sign(distance)) {

			t_move.createStraight(t_move.getPosition("x"), t_move.getPosition("y"), v, distance, ACCELERATION, t_move.calcStraight(t_move.v_data.elapsed));

		} else {

			printf("***************Error!!*****************\n");
			t_move.pubSignal(this->signal, -1);

		}

	}

}

void MovementAmountDesignation::Turn(double angle, double v) {

	double stack_angle = 0.0;
	//double start_angle = this->angle_to_quaternion(this->odom_data.angular_w, this->odom_data.angular_z);
	double start_angle = t_move.getPosition("agnle");


	double current_angle = 0.0;
	double before_angle = start_angle;
	double after_angle = start_angle;
	double angular_velocity = 0.0;

	t_move.createTurn(start_angle, v, angle, ANGLE_ACCELERATION, 0.5, 0.4);

	ros::Rate loop_rate(8);

	while (ros::ok()) {

		t_move.update();

		//current_angle = this->angle_to_quaternion(this->odom_data.angular_w, this->odom_data.angular_z);
		current_angle = t_move.getPosition("agnle");
		after_angle = current_angle;

		if (angle > 0) {

			angular_velocity = after_angle - before_angle;

			if (angular_velocity < -180) angular_velocity = after_angle + (360 - before_angle);

		} else {

			angular_velocity = before_angle - after_angle;

			if (angular_velocity < -180) angular_velocity = before_angle + (360 - after_angle);

		}

		stack_angle += angular_velocity;

		if (stack_angle >= abs(angle)) {
			t_move.pubVelocity(this->velocity, 0, 0, 0, 0);
			break;
		}

		loop_rate.sleep();

		//publish_twist(this->move, 0, this->calcTurn(stack_angle) * (angle == 0 ? 0 : angle / abs(angle)));
		t_move.pubVelocity(this->velocity, 0, 0, 2, t_move.calcTurn(stack_angle) * (angle == 0 ? 0 : angle / abs(angle)));

		printf("%f\n", stack_angle );

		before_angle = after_angle;

	}

	t_move.pubSignal(this->signal, 1);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementAmount");

	MovementAmountDesignation move;

	ros::spin();

	return 0;

}