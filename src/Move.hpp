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
	void pub_twist(const ros::Publisher& pub, double v, double a);
	void pub_signal(const ros::Publisher& pub, int signal);

	void set_flag(string s);
	bool get_flag(string s);
	double get_position(string element);
	double get_current_value(string s);
	void stop_move(string s);
	void set_odometry(const nav_msgs::Odometry::ConstPtr &odom);

	void create_straight(double x0, double y0, double max_velocity, double distance, double a, double v0);
	double straight(double point);
	void create_turn(double t0, double max_velocity, double angle, double a, double v0, double vn);
	double turn(double angular_point);

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
		double x0;
		double y0;
		double elapsed;
		double p1 = 0;
		double p2 = 0;
		double p3 = 0;
		double acceleration;
		double max_value;
		double initial_value;
		double current_value;
		double direction;
		bool flag = false;
	} VelocityProfile;

	typedef struct {
		double t0;
		double elapsed;
		double p1 = 0;
		double p2 = 0;
		double p3 = 0;
		double acceleration;
		double max_value;
		double initial_value;
		double deathbed_value;
		double current_value;
		double direction;
		bool flag = false;
	} AngleProfile;

	Odometry odom_data;
	VelocityProfile v_data;
	AngleProfile a_data;

};

//////////////////////////////////////////////////////////////////////////////
//public

Move::Move() {

	printf("start class of 'Move'\n");


}

Move::~Move() {

	printf("shutdown class of 'Move'\n");
}

//送信系
void Move::pub_twist(const ros::Publisher& pub, double v, double a) {

	geometry_msgs::Twist twist;

	twist.linear.x = v;
	twist.angular.z = a;

	pub.publish(twist);

}

void Move::pub_signal(const ros::Publisher &pub, int signal) {

	std_msgs::Int32 i;

	i.data = signal;

	pub.publish(i);

}

//メンバ関数
void Move::set_flag(string s) {

	if (s == "straight") v_data.flag = true;
	if (s == "turn") a_data.flag = true;

}

bool Move::get_flag(string s) {

	if (s == "straight") return v_data.flag;
	if (s == "turn") return a_data.flag;

	return false;
}

double Move::get_current_value(string s) {

	if (s == "straight") return this->v_data.current_value;
	if (s == "turn") return this->a_data.current_value;

	return 0;
}

double Move::get_position(string element) {

	if (element == "x") return this->odom_data.x;
	if (element == "y") return this->odom_data.y;
	if (element == "angle") return this->angle_to_quaternion(this->odom_data.angular_w, this->odom_data.angular_z);

	return 0;

}

void Move::stop_move(string s) {

	printf("ストップ\n");

	this->v_data.current_value = 0;
	this->a_data.current_value = 0;
	if (s == "straight") this->v_data.flag = false;
	if (s == "turn")this->a_data.flag = false;

}

//直進方向
void Move::create_straight(double x0, double y0, double max_velocity, double distance, double a, double v0) {

	this->set_flag("straight");

	this->v_data.max_value = max_velocity;
	this->v_data.acceleration = a;
	this->v_data.initial_value = v0;
	this->v_data.x0 = x0;
	this->v_data.y0 = y0;
	this->v_data.direction = this->sign(distance);
	this->v_data.p3 = abs(distance);

	this->v_data.p1 = (this->v_data.max_value - this->v_data.initial_value) / this->v_data.acceleration;
	this->v_data.p2 = (this->v_data.acceleration * this->v_data.p3 - this->v_data.max_value) / this->v_data.acceleration;

	if (this->v_data.p1 + (this->v_data.p3 - this->v_data.p2) > this->v_data.p3) {
		this->v_data.p1 = this->v_data.p3 / 2;
		this->v_data.p2 = this->v_data.p3 / 2;
		printf("linear_short\n");
	}

	printf("%f   %f   %f\n", v_data.p1, v_data.p2, v_data.p3 );

}

double Move::straight(double point) {

	this->v_data.current_value = 0;

	if (this->get_flag("straight")) {
		if (point >= 0 && point < this->v_data.p1) {

			this->v_data.current_value = this->v_data.acceleration * point + this->v_data.initial_value;

		} else if (point >= this->v_data.p1 && point < this->v_data.p2) {

			this->v_data.current_value = this->v_data.max_value;

		} else if (point >= this->v_data.p2 && point < this->v_data.p3) {

			this->v_data.current_value = -1 * this->v_data.acceleration * point + this->v_data.acceleration * this->v_data.p3;

		} else {

			this->stop_move("straight");

		}
	}

	return this->v_data.current_value * this->v_data.direction;
}

//回転方向
void Move::create_turn(double t0, double max_velocity, double angle, double a, double v0, double vn) {

	this->set_flag("turn");

	this->a_data.max_value = max_velocity;
	this->a_data.acceleration = a;
	this->a_data.initial_value = v0;
	this->a_data.deathbed_value = vn;
	this->a_data.t0 = t0;
	this->a_data.direction = this->sign(angle);
	this->a_data.p3 = abs(angle);

	this->a_data.p1 = (this->a_data.max_value - this->a_data.initial_value) / this->a_data.acceleration;
	this->a_data.p2 = ((this->a_data.initial_value + this->a_data.acceleration * this->a_data.p3) - this->a_data.max_value) / this->a_data.acceleration;

	if (this->a_data.p1 + (this->a_data.p3 - this->a_data.p2) > this->a_data.p3) {
		this->a_data.p1 = this->a_data.p3 / 2;
		this->a_data.p2 = this->a_data.p3 / 2;
		printf("anguler_short\n");
	}

	printf("%f   %f   %f\n", a_data.p1, a_data.p2, a_data.p3 );

}

double Move::turn(double angular_point) {

	this->a_data.current_value = 0;

	if (this->get_flag("turn")) {
		if (angular_point >= 0 && angular_point < this->a_data.p1) {

			this->a_data.current_value = this->a_data.acceleration * angular_point + this->a_data.initial_value;

		} else if (angular_point >= this->a_data.p1 && angular_point < this->a_data.p2) {

			this->a_data.current_value = this->a_data.max_value;

		} else if (angular_point >= this->a_data.p2 && angular_point < this->a_data.p3) {

			this->a_data.current_value = (-1 * this->a_data.acceleration * angular_point) + this->a_data.initial_value + (this->a_data.acceleration * this->a_data.p3);

		} else {

			this->stop_move("turn");

		}
	}

	return this->a_data.current_value * this->a_data.direction;
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