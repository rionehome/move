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


void callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {

	//[0]:liner距離 , [1]:liner速度 , [2]:angle距離 , [3]:angle速度

	if (msg->data[2] == 0) { //linter

		this->straight(msg->data[0], msg->data[1]);

	} else if (msg->data[0] == 0) { //angle

		this->turn(msg->data[2], msg->data[3]);

	} else {

		printf("error\n");

	}

}

void publish_twist(const ros::Publisher& pub, double x, double az) {

	geometry_msgs::Twist twist;

	twist.linear.x = x;
	twist.angular.z = az;

	pub.publish(twist);

}

void pub_signal(int s) {

	std_msgs::Int32 msg;

	msg.data = s;
	
	signal.publish(msg);

}


void straight(double distance, double v) {

	if (v_data.p3 == 0) {

		ros::Rate loop_rate(24);

		this->acceleration_create(this->odom_data.x, this->odom_data.y, v, distance, ACCELERATION, 0.08);

		while (ros::ok()) {

			this->updata();

			v_data.elapsed = hypot(this->odom_data.x - v_data.x0, this->odom_data.y - v_data.y0);

			if ( v_data.elapsed > abs(v_data.p3)) break;

			loop_rate.sleep();

			printf("%f\n", v_data.elapsed );

			if (distance > 0) {
				this->publish_twist(this->move, this->acceleration_function(v_data.elapsed), 0);
			} else {
				this->publish_twist(this->move, -1 * this->acceleration_function(v_data.elapsed), 0);
			}

		}

		v_data.p3 = 0;
		this->pub_signal(1);

	} else {

		if (this->sign(v_data.distance) == this->sign(distance)) {

			this->acceleration_create(this->odom_data.x, this->odom_data.y, v, distance, ACCELERATION, this->acceleration_function(v_data.elapsed));

		} else {

			printf("***************Error!!*****************\n");
			this->pub_signal(-1);

		}

	}

}

void turn(double angle, double v) {

	double stack_angle = 0.0;
	double start_angle = this->angle_to_quaternion(this->odom_data.angular_w, this->odom_data.angular_z);
	double current_angle = 0.0;
	double before_angle = start_angle;
	double after_angle = start_angle;
	double angular_velocity = 0.0;

	this->angular_acceleration_create(start_angle, v, angle, ANGLE_ACCELERATION, 0.5, 0.4);

	ros::Rate loop_rate(24);

	while (ros::ok()) {

		this->updata();

		current_angle = this->angle_to_quaternion(this->odom_data.angular_w, this->odom_data.angular_z);

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
			publish_twist(this->move, 0, 0);
			break;
		}

		loop_rate.sleep();

		publish_twist(this->move, 0, this->angular_acceleration_function(stack_angle) * (angle == 0 ? 0 : angle / abs(angle)));

		printf("%f\n", stack_angle );

		before_angle = after_angle;

	}

	this->pub_signal(1);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementAmount");

	Move move;

	

	ros::spin();

	return 0;

}