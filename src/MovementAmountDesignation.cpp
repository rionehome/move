#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <time.h>
#include <stdio.h>
#include <math.h>

using namespace std;

class MovementAmountDesignation {
private:
	ros::NodeHandle n;
	ros::Subscriber odom;
	ros::Subscriber amount_distance;
	ros::Subscriber amount_angle;
	ros::Publisher move;

	double x;
	double y;
	double z;
	double angular_x;
	double angular_y;
	double angular_z;
	double angular_w;

	double d1 = 0;
	double d2 = 0;
	double d3 = 0;
	double acceleration = 2;

	void odometry(const nav_msgs::Odometry::ConstPtr &odom);
	void line(const std_msgs::Float64::ConstPtr &msg);
	void curve(const std_msgs::Float64::ConstPtr &msg);
	void straight(double distance);
	void turn(double angle);
	void acceleration_create(double max_velocity, double distance);
	double acceleration_function(double point, double max_velocity);
	void angular_acceleration_create(double max_angular_velocity, double angular);
	double angular_acceleration_function(double angular_point, double max_angular);


public:
	MovementAmountDesignation();
	~MovementAmountDesignation();

	void publish_twist(const ros::Publisher& pub, double x, double az);
	void update() {ros::spinOnce();}
	void calc() {while (1) {update();}}
	double angle_to_quaternion(double w, double z);
	double rad_to_quaternion(double w, double z);
	double angle_to_rad(double rad) {return rad * 180 / M_PI;}
	double rad_to_angle(double angle) {return (angle * M_PI) / 180;}
};

MovementAmountDesignation::MovementAmountDesignation() {

	printf("start class of 'MovementAmountDesignation'\n");

	this->odom = n.subscribe("/odom", 1000, &MovementAmountDesignation::odometry, this);
	this->amount_distance = n.subscribe("/move/distance", 1000, &MovementAmountDesignation::line, this);
	this->amount_angle = n.subscribe("/move/angle", 1000, &MovementAmountDesignation::curve, this);
	this->move = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

}


MovementAmountDesignation::~MovementAmountDesignation() {

	printf("shutdown class of 'MovementAmountDesignation'\n");
}

void MovementAmountDesignation::publish_twist(const ros::Publisher& pub, double x, double az) {

	geometry_msgs::Twist twist;

	twist.linear.x = x;
	twist.angular.z = az;

	pub.publish(twist);

}

void MovementAmountDesignation::odometry(const nav_msgs::Odometry::ConstPtr &odom) {

	this->x = odom->pose.pose.position.x;
	this->y = odom->pose.pose.position.y;
	this->z = odom->pose.pose.position.z;
	this->angular_x = odom->pose.pose.orientation.x;
	this->angular_y = odom->pose.pose.orientation.y;
	this->angular_z = odom->pose.pose.orientation.z;
	this->angular_w = odom->pose.pose.orientation.w;
}

void MovementAmountDesignation::line(const std_msgs::Float64::ConstPtr &msg) {
	this->straight(msg->data);
}

void MovementAmountDesignation::curve(const std_msgs::Float64::ConstPtr &msg) {
	this->turn(msg->data);
}

double MovementAmountDesignation::angle_to_quaternion(double w, double z) {
	return abs((z > 0 ? 1 : 360) - this->angle_to_rad(acos(w) * 2));
}

double MovementAmountDesignation::rad_to_quaternion(double w, double z) {
	return acos(this->angular_w) * (this->angular_z > 0 ? 1 : -1) * 2;
}

void MovementAmountDesignation::acceleration_create(double max_velocity, double distance) {

	this->d1 = (max_velocity - 0.08) / acceleration;
	this->d2 = (acceleration * distance - max_velocity) / acceleration;

	if (this->d1 + this->d2 > distance) {
		this->d1 = distance / 2;
		this->d2 = distance / 2;
	}

	this->d3 = distance;

}

double MovementAmountDesignation::acceleration_function(double point, double max_velocity) {

	double result = 0.0;

	if (point >= 0 && point < this->d1) {
		printf("d1\n");
		result = acceleration * point + 0.08;
		printf("%f\n", result);
		return result;
	}

	if (point >= d1 && point < this->d2) {
		printf("d2\n");
		return max_velocity;
	}

	if (point >= d2 && point < this->d3) {
		result = -acceleration * point + acceleration * this->d3;
		printf("d3\n");
		printf("%f\n", result);
		return result;
	}

	printf("d4\n");
	return 0.1;
}



void MovementAmountDesignation::angular_acceleration_create(double max_velocity, double distance) {

	this->d1 = (max_velocity - 0.08) / acceleration;
	this->d2 = (acceleration * distance - max_velocity) / acceleration;

	if (this->d1 + this->d2 > distance) {
		this->d1 = distance / 2;
		this->d2 = distance / 2;
		printf("short\n");
	}

	this->d3 = distance;

}

double MovementAmountDesignation::angular_acceleration_function(double angular_point, double max_angular) {

	double result = 0.0;


	printf("d4\n");
	return 0.1;
}

void MovementAmountDesignation::straight(double distance) {

	double start_point_x = this->x;
	double start_point_y = this->y;
	double elapsed = 0;

	ros::Rate loop_rate(20);

	this->acceleration_create(0.3, abs(distance));

	while (ros::ok()) {

		this->update();

		elapsed = hypot(this->x - start_point_x, this->y - start_point_y);

		if ( elapsed > abs(distance)) break;

		loop_rate.sleep();

		printf("%f\n", elapsed );

		if (distance > 0) {
			this->publish_twist(this->move, this->acceleration_function(elapsed, 0.3), 0);
		} else {
			this->publish_twist(this->move, -1 * this->acceleration_function(elapsed, 0.3), 0);
		}

	}
}

void MovementAmountDesignation::turn(double angle) {

	double stack_angle = 0.0;
	double start_angle = this->angle_to_quaternion(this->angular_w, this->angular_z);
	double current_angle = 0.0;
	double before_angle = start_angle;
	double after_angle = start_angle;
	double angular_velocity = 0.0;

	if (angle > 0) {

		ros::Rate loop_rate(20);

		while (ros::ok()) {

			this->update();

			current_angle = this->angle_to_quaternion(this->angular_w, this->angular_z);

			after_angle = current_angle;

			angular_velocity = after_angle - before_angle;

			if (angular_velocity < -180) angular_velocity = after_angle + (360 - before_angle);

			stack_angle = angular_velocity + stack_angle;

			if (stack_angle > angle) break;

			loop_rate.sleep();

			publish_twist(this->move, 0, 0.5);

			before_angle = after_angle;

		}

	} else {

		ros::Rate loop_rate(20);

		while (ros::ok()) {

			this->update();

			current_angle = this->angle_to_quaternion(this->angular_w, this->angular_z);

			after_angle = current_angle;

			angular_velocity = before_angle - after_angle;

			if (angular_velocity < -180) angular_velocity = before_angle + (360 - after_angle);

			stack_angle = angular_velocity + stack_angle;

			if (stack_angle > abs(angle)) break;

			loop_rate.sleep();

			publish_twist(this->move, 0, -0.5);

			before_angle = after_angle;

		}

	}

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementAmount");

	MovementAmountDesignation *move = new MovementAmountDesignation();

	move->calc();

	ros::spin();

	return 0;

}