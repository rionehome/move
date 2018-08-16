#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>

#define ACCELERATION 2
#define ANGLE_ACCELERATION 0.015

using namespace std;

class MovementAmountDesignation {
private:
	ros::NodeHandle n;
	ros::Subscriber odom;
	ros::Subscriber amount_distance;
	ros::Subscriber amount_angle;
	ros::Subscriber calc_move;
	ros::Publisher move;
	ros::Publisher signal;

	typedef struct {
		double x;
		double y;
		double z;
		double angular_x;
		double angular_y;
		double angular_z;
		double angular_w;
	} Odometry;

	typedef struct {
		double x0 = 0;
		double y0 = 0;
		double elapsed = 0;
		double p1 = 0;
		double p2 = 0;
		double p3 = 0;
		double acceleration = 0;
		double v = 0;
		double initial_value = 0;
		double distance = 0;
	} VelocityProfile;

	typedef struct {
		double t0 = 0;
		double elapsed = 0;
		double p1 = 0;
		double p2 = 0;
		double p3 = 0;
		double acceleration = 0;
		double v = 0;
		double initial_value = 0;
		double deathbed_value = 0;
		double angle = 0;
	} AngleProfile;

	Odometry odom_data;
	VelocityProfile v_data;
	AngleProfile a_data;

	void odometry(const nav_msgs::Odometry::ConstPtr &odom);
	void callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
	void straight(double distance, double v);
	void turn(double angle, double v);
	void acceleration_create(double x0, double y0, double max_velocity, double distance, double a, double v0);
	double acceleration_function(double point);
	void angular_acceleration_create(double t0, double max_velocity, double angle, double a, double v0, double vn);
	double angular_acceleration_function(double angular_point);


public:
	MovementAmountDesignation();
	~MovementAmountDesignation();

	void publish_twist(const ros::Publisher& pub, double x, double az);
	void pub_signal(int s);
	void updata() {ros::spinOnce();}
	double angle_to_quaternion(double w, double z) {
		return abs((z > 0 ? 1 : 360) - this->angle_to_rad(acos(w) * 2));
	}
	double rad_to_quaternion(double w, double z) {
		return acos(this->odom_data.angular_w) * (this->odom_data.angular_z > 0 ? 1 : -1) * 2;
	}
	double angle_to_rad(double rad) {return rad * 180 / M_PI;}
	double rad_to_angle(double angle) {return (angle * M_PI) / 180;}
	double sign(double A) {return  A == 0 ? 0 : A / abs(A);}
};

MovementAmountDesignation::MovementAmountDesignation() {

	printf("start class of 'MovementAmountDesignation'\n");

	this->odom = n.subscribe("/odom", 1000, &MovementAmountDesignation::odometry, this);
	this->calc_move = n.subscribe("/move/amount", 1000, &MovementAmountDesignation::callback, this);
	this->move = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	this->signal = n.advertise<std_msgs::Int32 >("/move/signal", 1000);

}


MovementAmountDesignation::~MovementAmountDesignation() {

	printf("shutdown class of 'MovementAmountDesignation'\n");
}

void MovementAmountDesignation::callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {

	//[0]:liner距離 , [1]:liner速度 , [2]:angle距離 , [3]:angle速度

	if (msg->data[2] == 0) { //linter

		this->straight(msg->data[0], msg->data[1]);

	} else if (msg->data[0] == 0) { //angle

		this->turn(msg->data[2], msg->data[3]);

	} else {

		printf("error\n");

	}

}

void MovementAmountDesignation::publish_twist(const ros::Publisher& pub, double x, double az) {

	geometry_msgs::Twist twist;

	twist.linear.x = x;
	twist.angular.z = az;

	pub.publish(twist);

}

void MovementAmountDesignation::pub_signal(int s) {

	std_msgs::Int32 msg;

	msg.data = s;
	
	signal.publish(msg);

}

void MovementAmountDesignation::odometry(const nav_msgs::Odometry::ConstPtr &odom) {

	this->odom_data.x = odom->pose.pose.position.x;
	this->odom_data.y = odom->pose.pose.position.y;
	this->odom_data.z = odom->pose.pose.position.z;
	this->odom_data.angular_x = odom->pose.pose.orientation.x;
	this->odom_data.angular_y = odom->pose.pose.orientation.y;
	this->odom_data.angular_z = odom->pose.pose.orientation.z;
	this->odom_data.angular_w = odom->pose.pose.orientation.w;

}

void MovementAmountDesignation::acceleration_create(double x0, double y0, double max_velocity, double distance, double a, double v0) {

	this->v_data.v = max_velocity;
	this->v_data.acceleration = a;
	this->v_data.initial_value = v0;
	this->v_data.x0 = x0;
	this->v_data.y0 = y0;
	this->v_data.distance = distance;
	this->v_data.p3 = abs(distance);

	this->v_data.p1 = (this->v_data.v - this->v_data.initial_value) / this->v_data.acceleration;
	this->v_data.p2 = (this->v_data.acceleration * this->v_data.p3 - this->v_data.v) / this->v_data.acceleration;

	if (this->v_data.p1 + (this->v_data.p3 - this->v_data.p2) > this->v_data.p3) {
		this->v_data.p1 = this->v_data.p3 / 2;
		this->v_data.p2 = this->v_data.p3 / 2;
		printf("short\n");
	}

	printf("%f   %f   %f\n", v_data.p1, v_data.p2, v_data.p3 );

}

double MovementAmountDesignation::acceleration_function(double point) {

	double result = 0.0;

	if (point >= 0 && point < this->v_data.p1) {
		printf("d1\n");
		result = this->v_data.acceleration * point + this->v_data.initial_value;
		printf("%f\n", result);
		return result;
	}

	if (point >= this->v_data.p1 && point < this->v_data.p2) {
		printf("d2\n");
		return this->v_data.v;
	}

	if (point >= this->v_data.p2 && point < this->v_data.p3) {
		result = -1 * this->v_data.acceleration * point + this->v_data.acceleration * this->v_data.p3;
		printf("d3\n");
		printf("%f\n", result);
		return result;
	}

	printf("d4\n");
	return 0.1;
}


void MovementAmountDesignation::straight(double distance, double v) {

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

void MovementAmountDesignation::angular_acceleration_create(double t0, double max_velocity, double angle, double a, double v0, double vn) {

	this->a_data.v = max_velocity;
	this->a_data.acceleration = a;
	this->a_data.initial_value = v0;
	this->a_data.deathbed_value = vn;
	this->a_data.t0 = t0;
	this->a_data.angle = angle;
	this->a_data.p3 = abs(angle);

	this->a_data.p1 = (this->a_data.v - this->a_data.initial_value) / this->a_data.acceleration;
	this->a_data.p2 = ((this->a_data.initial_value + this->a_data.acceleration * this->a_data.p3) - this->a_data.v) / this->a_data.acceleration;

	if (this->a_data.p1 + (this->a_data.p3 - this->a_data.p2) > this->a_data.p3) {
		this->a_data.p1 = this->a_data.p3 / 2;
		this->a_data.p2 = this->a_data.p3 / 2;
		printf("short\n");
	}

	printf("%f   %f   %f\n", a_data.p1, a_data.p2, a_data.p3 );

}

double MovementAmountDesignation::angular_acceleration_function(double angular_point) {

	double result = 0.0;

	if (angular_point >= 0 && angular_point < this->a_data.p1) {
		printf("a_d1\n");
		result = this->a_data.acceleration * angular_point + this->a_data.initial_value;
		printf("%f\n", result);
		return result;
	}

	if (angular_point >= this->a_data.p1 && angular_point < this->a_data.p2) {
		printf("a_d2\n");
		return this->a_data.v;
	}

	if (angular_point >= this->a_data.p2 && angular_point < this->a_data.p3) {
		result = (-1 * this->a_data.acceleration * angular_point) + this->a_data.initial_value + (this->a_data.acceleration * this->a_data.p3);
		printf("a_d3\n");
		printf("result %f\n", result);
		return result;
	}

	printf("a_d4\n");
	return 0.45;
}

void MovementAmountDesignation::turn(double angle, double v) {

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

	MovementAmountDesignation move;

	ros::spin();

	return 0;

}