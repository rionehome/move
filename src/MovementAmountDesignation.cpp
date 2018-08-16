#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include "Move.hpp"

#define ACCELERATION 2
#define ANGLE_ACCELERATION 0.015

using namespace std;

Move m;

class MovementAmountDesignation {
public:
	MovementAmountDesignation();
	~MovementAmountDesignation();

	ros::NodeHandle n;
	ros::Subscriber odom;
	ros::Subscriber amout_move;
	ros::Publisher pub_move;
	ros::Publisher signal;

	double stack_distance = 0;
	double stack_angle = 0;

	void calc(const std_msgs::Float64MultiArray::ConstPtr &msgs);
	void updata();

private:

	double linear_current_value = 0;
	double angular_current_value = 0;

	void set_odom(const nav_msgs::Odometry::ConstPtr &odom) {m.set_odometry(odom);}

};


///////////////////////////////////////////////////////////////////////////////////

MovementAmountDesignation::MovementAmountDesignation() {

	printf("start class of 'MovementAmountDesignation'\n");

	this->odom = n.subscribe("/odom", 1000, &MovementAmountDesignation::set_odom, this);
	this->amout_move = n.subscribe("/move/amount", 1000, &MovementAmountDesignation::calc, this);
	this->pub_move = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	this->signal = n.advertise<std_msgs::Int32 >("/move/signal", 1000);

}


MovementAmountDesignation::~MovementAmountDesignation() {

	printf("shutdown class of 'MovementAmountDesignation'\n");
}



void MovementAmountDesignation::calc(const std_msgs::Float64MultiArray::ConstPtr &msgs) {

	if (linear_current_value == 0 && angular_current_value == 0) { //静止の場合

		this->stack_distance = 0;
		this->stack_angle = 0;

		m.create_straight(m.get_position("x"), m.get_position("y"), msgs->data[1], msgs->data[0], ACCELERATION, 0.1);
		m.create_turn(m.get_position("angle"), msgs->data[3], msgs->data[2], ANGLE_ACCELERATION, 0.5, 0.4);

	} else {

		printf("動作中\n");

	}

}


void MovementAmountDesignation::updata() {

	m.updata();

	this->linear_current_value = m.get_current_value("straight");
	this->angular_current_value = m.get_current_value("turn");

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementAmount");

	MovementAmountDesignation amount;

	double before_point[2];
	double before_angle = 0;
	double after_point[2];
	double after_angle = 0;
	double angular_velocity;

	ros::Rate loop_rate(24);

	while (ros::ok()) {

		amount.updata();

		before_point[0] = m.get_position("x");
		before_point[1] = m.get_position("y");
		before_angle = m.get_position("angle");

		amount.stack_distance += hypot(before_point[0] - after_point[0], before_point[1] - after_point[1]);

		angular_velocity = abs(after_angle - before_angle);

		if (angular_velocity > 180 && after_angle < before_angle) angular_velocity = after_angle + (360 - before_angle);
		if (angular_velocity > 180 && after_angle > before_angle) angular_velocity = before_angle + (360 - after_angle);

		printf("%f   %f\n", angular_velocity, before_angle );

		amount.stack_angle += angular_velocity;

		m.pub_twist(amount.pub_move, m.straight(amount.stack_distance), m.turn(amount.stack_angle)); //送信

		after_point[0] = before_point[0];
		after_point[1] = before_point[1];
		after_angle = before_angle;

		loop_rate.sleep();

	}

	return 0;

}