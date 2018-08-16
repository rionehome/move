#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>

#ifndef   C_HPP
#define   C_HPP

using namespace std;

class Move {
public:
	Move();
	~Move();

	void updata() {ros::spinOnce();}
	void create_straight(double x0, double y0, double max_velocity, double distance, double a, double v0);
	double straight(double point);
	void create_turn(double t0, double max_velocity, double angle, double a, double v0, double vn);
	double turn(double angular_point);
	bool get_flag(string s);
	void set_flag(string s);

	double angle_to_quaternion(double w, double z) {
		return abs((z > 0 ? 1 : 360) - this->angle_to_rad(acos(w) * 2));
	}
	double rad_to_quaternion(double w, double z) {
		return acos(this->odom_data.angular_w) * (this->odom_data.angular_z > 0 ? 1 : -1) * 2;
	}
	double angle_to_rad(double rad) {return rad * 180 / M_PI;}
	double rad_to_angle(double angle) {return (angle * M_PI) / 180;}
	double sign(double A) {return  A == 0 ? 0 : A / abs(A);}


private:
	ros::NodeHandle n;
	ros::Subscriber odom;

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
		bool flag = false;
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
		bool flag = false;
	} AngleProfile;

	Odometry odom_data;
	VelocityProfile v_data;
	AngleProfile a_data;

	void set_odometry(const nav_msgs::Odometry::ConstPtr &odom);

};

//////////////////////////////////////////////////////////////////////////////
//public

Move::Move() {

	printf("start class of 'Move'\n");
	this->odom = n.subscribe("/odom", 1000, &Move::set_odometry, this);

}

Move::~Move() {

	printf("shutdown class of 'Move'\n");
}

//直進方向
void Move::create_straight(double x0, double y0, double max_velocity, double distance, double a, double v0) {

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

double Move::straight(double point) {

	double result = 0.0;

	this->set_flag("straight");

	if (point >= 0 && point < this->v_data.p1) {
		result = this->v_data.acceleration * point + this->v_data.initial_value;
		return result;
	}

	if (point >= this->v_data.p1 && point < this->v_data.p2) {
		return this->v_data.v;
	}

	if (point >= this->v_data.p2 && point < this->v_data.p3) {
		result = -1 * this->v_data.acceleration * point + this->v_data.acceleration * this->v_data.p3;
		return result;
	}

	printf("d4\n");
	return 0.1;
}

//回転方向
void Move::create_turn(double t0, double max_velocity, double angle, double a, double v0, double vn) {

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

double Move::turn(double angular_point) {

	double result = 0.0;

	this->set_flag("turn");

	if (angular_point >= 0 && angular_point < this->a_data.p1) {
		result = this->a_data.acceleration * angular_point + this->a_data.initial_value;
		return result;
	}

	if (angular_point >= this->a_data.p1 && angular_point < this->a_data.p2) {
		return this->a_data.v;
	}

	if (angular_point >= this->a_data.p2 && angular_point < this->a_data.p3) {
		result = (-1 * this->a_data.acceleration * angular_point) + this->a_data.initial_value + (this->a_data.acceleration * this->a_data.p3);
		return result;
	}

	printf("a_d4\n");
	return 0.45;
}

void Move::set_flag(string s) {

	if (s == "straight") v_data.flag = true;
	if (s == "turn") a_data.flag = true;
}

bool Move::get_flag(string s) {

	if (s == "straight") return v_data.flag;

	if (s == "turn") return a_data.flag;

	return false;
}


////////////////////////////////////////////////////////////////////////////////////////
//private

void Move::set_odometry(const nav_msgs::Odometry::ConstPtr &odom) {

	this->odom_data.x = odom->pose.pose.position.x;
	this->odom_data.y = odom->pose.pose.position.y;
	this->odom_data.z = odom->pose.pose.position.z;
	this->odom_data.angular_x = odom->pose.pose.orientation.x;
	this->odom_data.angular_y = odom->pose.pose.orientation.y;
	this->odom_data.angular_z = odom->pose.pose.orientation.z;
	this->odom_data.angular_w = odom->pose.pose.orientation.w;

}


#endif