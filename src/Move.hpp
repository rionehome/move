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

	void updata();
	void velocity_updata();
	void pub_twist(const ros::Publisher& pub, double v, double a);
	void pub_signal(const ros::Publisher& pub, int signal);

	void set_flag(string s);
	bool get_flag(string s);
	double get_position(string element);
	void reset_progress(string s);
	void add_progress(string s, double value);
	double get_progress(string s);
	double get_velocity(string s);
	double get_variate(string s);
	void stop_move(string s);
	void set_odometry(const nav_msgs::Odometry::ConstPtr &odom);

	void create_straight(double max_velocity, double distance, double a, double v0);
	double straight(double point);
	void create_turn(double max_velocity, double angle, double a, double v0, double vn);
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
	//[0]:before, [1]:after
	double angle_delta[2] = {0, 0};
	double x_delta[2] = {0, 0};
	double y_delta[2] = {0, 0};

	typedef struct {
		double x;
		double y;
		double z;
		double angular_x;
		double angular_y;
		double angular_z;
		double angular_w;
		double linear_speed;
		double angular_speed;
	} Odometry;

	typedef struct {
		double x0;
		double y0;
		double elapsed;
		double p1 = 0;
		double p2 = 0;
		double p3 = 0;
		double p4 = 0;
		double p5 = 0;
		double p6 = 0;
		double distance;
		double acceleration;
		double max_value;
		double initial_value;
		double stack_distance;
		bool flag = false;
	} VelocityProfile;

	typedef struct {
		double t0;
		double elapsed;
		double p1 = 0;
		double p2 = 0;
		double p3 = 0;
		double p4 = 0;
		double p5 = 0;
		double p6 = 0;
		double angle;
		double acceleration;
		double max_value;
		double initial_value;
		double deathbed_value;
		double stack_angle;
		bool flag = false;
	} AngleProfile;

	//ros::NodeHandle n;
	//ros::Publisher signal;

	Odometry odom_data;
	VelocityProfile v_data;
	AngleProfile a_data;

};

//////////////////////////////////////////////////////////////////////////////
//public

Move::Move() {

	printf("start class of 'Move'\n");

	//this->signal = n.advertise<std_msgs::Int32 >("/move/signal", 1000);
}

Move::~Move() {

	printf("shutdown class of 'Move'\n");
}

//更新系////
void Move::velocity_updata() { //速度

	angle_delta[0] = angle_delta[1];
	x_delta[0] = x_delta[1];
	y_delta[0] = y_delta[1];

	x_delta[1] = Move::get_position("x");
	y_delta[1] = Move::get_position("y");
	angle_delta[1] = Move::get_position("angle");

}

void Move::updata() {

	ros::spinOnce(); //ソケットメッセージの初期化

	Move::velocity_updata(); //速度更新

}

//送信系///
void Move::pub_twist(const ros::Publisher& pub, double v, double a) {

	geometry_msgs::Twist twist;

	twist.linear.x = v;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = a;

	pub.publish(twist);

}

void Move::pub_signal(const ros::Publisher &pub, int signal) {

	std_msgs::Int32 i;

	i.data = signal;

	pub.publish(i);

}

//メンバ関数////

void Move::set_flag(string s) {

	if (s == "straight") v_data.flag = true;
	if (s == "turn") a_data.flag = true;

}

bool Move::get_flag(string s) {

	if (s == "straight") return v_data.flag;
	if (s == "turn") return a_data.flag;

	return false;
}
//現在の座標を取得
double Move::get_position(string element) {

	if (element == "x") return this->odom_data.x;
	if (element == "y") return this->odom_data.y;
	if (element == "angle") return this->angle_to_quaternion(this->odom_data.angular_w, this->odom_data.angular_z);

	return 0;

}
//現在の速度を取得
double Move::get_velocity(string s) {

	if (s == "straight") {
		if (abs(this->odom_data.linear_speed) > 0.01) return this->odom_data.linear_speed;
	}

	if (s == "turn") {
		if (abs(this->odom_data.angular_speed) > 0.01) return this->odom_data.angular_speed;
	}

	return 0;
}
//現在の変量を取得
double Move::get_variate(string s) {

	double angle_velocity = 0;
	if (s == "straight") {
		return hypot(x_delta[1] - x_delta[0], y_delta[1] - y_delta[0]);
	}

	if (s == "turn") {
		angle_velocity = abs(angle_delta[1] - angle_delta[0]);
		if (angle_velocity > 180 && angle_delta[1] < angle_delta[0]) angle_velocity = angle_delta[1] + (360 - angle_delta[0]);
		if (angle_velocity > 180 && angle_delta[1] > angle_delta[0]) angle_velocity = angle_delta[0] + (360 - angle_delta[1]);
		return angle_velocity;
	}

	return 0;
}
//移動距離を初期化
void Move::reset_progress(string s) {

	if (s == "straight") this->v_data.stack_distance = 0;
	if (s == "turn") this->a_data.stack_angle = 0;

}
//移動距離を加算
void Move::add_progress(string s, double value) {

	if (s == "straight") this->v_data.stack_distance += value;
	if (s == "turn") this->a_data.stack_angle += value;

}
//移動距離を取得
double Move::get_progress(string s) {

	if (s == "straight") return this->v_data.stack_distance;
	if (s == "turn") return this->a_data.stack_angle;

	return 0;
}

void Move::stop_move(string s) {

	if (s == "straight") {
		printf("straight-ストップ\n");
		this->v_data.flag = false;
		//pub_signal(this->signal, 1);
	}
	if (s == "turn") {
		printf("turn-ストップ\n");
		this->a_data.flag = false;
		//pub_signal(this->signal, 1);
	}

}

//直進方向
void Move::create_straight(double max_velocity, double distance, double a, double v0) {

	this->set_flag("straight");

	this->v_data.max_value = abs(max_velocity);
	this->v_data.acceleration = abs(a);
	this->v_data.initial_value = abs(v0);
	this->v_data.p3 = abs(distance);
	this->v_data.distance = distance;

	this->v_data.p1 = (this->v_data.max_value - this->v_data.initial_value) / this->v_data.acceleration;
	this->v_data.p2 = (this->v_data.acceleration * abs(this->v_data.distance) - this->v_data.max_value) / this->v_data.acceleration;

	if (this->v_data.p1 + (this->v_data.p3 - this->v_data.p2) > this->v_data.p3) {
		this->v_data.p1 = 0;
		this->v_data.p2 = this->v_data.p3 / 2;
		printf("linear_short\n");
	}

	printf("straight %f   %f   %f\n", v_data.p1, v_data.p2, v_data.p3);

}

double Move::straight(double point) {

	double result = 0;

	if (this->get_flag("straight")) {

		if (point >= this->v_data.p5 && point < this->v_data.p6) {

			printf("d6\n");
			return result * this->sign(this->get_velocity("straight") == 0 ? this->v_data.distance : this->get_velocity("straight"));

		} else if (point >= this->v_data.p4 && point < this->v_data.p5) {

			printf("d5\n");
			return result * this->sign(this->get_velocity("straight") == 0 ? this->v_data.distance : this->get_velocity("straight"));

		} else if (point >= this->v_data.p3 && point < this->v_data.p4) {

			printf("d4\n");
			return result * this->sign(this->get_velocity("straight") == 0 ? this->v_data.distance : this->get_velocity("straight"));

		} else if (point >= this->v_data.p2 && point < this->v_data.p3) {

			result = -1 * this->v_data.acceleration * point + this->v_data.acceleration * this->v_data.p3;
			printf("d3\n");
			return result * this->sign(this->get_velocity("straight") == 0 ? this->v_data.distance : this->get_velocity("straight"));

		} else if (point >= this->v_data.p1 && point < this->v_data.p2) {

			result = this->v_data.max_value;
			printf("d2\n");
			return result * this->sign(this->get_velocity("straight") == 0 ? this->v_data.distance : this->get_velocity("straight"));

		} else if (point >= 0 && point < this->v_data.p1) {

			result = this->v_data.acceleration * point + this->v_data.initial_value;
			printf("d1\n");
			return result * this->sign(this->get_velocity("straight") == 0 ? this->v_data.distance : this->get_velocity("straight"));

		} else {

			this->stop_move("straight");

		}
	}

	return result;
}

//回転方向
void Move::create_turn(double max_velocity, double angle, double a, double v0, double vn) {

	this->set_flag("turn");

	this->a_data.max_value = abs(max_velocity);
	this->a_data.acceleration = abs(a);
	this->a_data.initial_value = abs(v0);
	this->a_data.deathbed_value = abs(vn);
	this->a_data.p3 = abs(angle);
	this->a_data.angle = angle;

	this->a_data.p1 = (this->a_data.max_value - this->a_data.initial_value) / this->a_data.acceleration;
	this->a_data.p2 = ((this->a_data.deathbed_value + this->a_data.acceleration * abs(this->a_data.angle)) - this->a_data.max_value) / this->a_data.acceleration;

	printf("%f - %f\n", this->a_data.acceleration * this->a_data.angle, this->a_data.max_value );

	if (this->a_data.p1 + (this->a_data.p3 - this->a_data.p2) > this->a_data.p3) {
		this->a_data.p1 = 0;
		this->a_data.p2 = (this->a_data.deathbed_value + ((this->a_data.acceleration * this->a_data.p3) - this->a_data.initial_value)) / (2 * this->a_data.acceleration);
		printf("anguler_short\n");
	}

	printf("%f   %f   %f\n", a_data.p1, a_data.p2, a_data.p3 );

}

double Move::turn(double angular_point) {

	double result = 0;

	if (this->get_flag("turn")) {

		if (angular_point >= this->a_data.p5 && angular_point < this->a_data.p6) {

			printf("a6\n");
			return result * this->sign(this->get_velocity("turn") == 0 ? this->a_data.angle : this->get_velocity("turn"));

		} else if (angular_point >= this->a_data.p4 && angular_point < this->a_data.p5) {

			printf("a5\n");
			return result * this->sign(this->get_velocity("turn") == 0 ? this->a_data.angle : this->get_velocity("turn"));

		} else if (angular_point >= this->a_data.p3 && angular_point < this->a_data.p4) {

			printf("a4\n");
			return result * this->sign(this->get_velocity("turn") == 0 ? this->a_data.angle : this->get_velocity("turn"));

		} else if (angular_point >= this->a_data.p2 && angular_point < this->a_data.p3) {

			result = (-1 * this->a_data.acceleration * angular_point) + this->a_data.deathbed_value + (this->a_data.acceleration * this->a_data.p3);
			printf("a3\n");
			return result * this->sign(this->get_velocity("turn") == 0 ? this->a_data.angle : this->get_velocity("turn"));

		} else if (angular_point >= this->a_data.p1 && angular_point < this->a_data.p2) {

			result = this->a_data.max_value;
			printf("a2\n");
			return result * this->sign(this->get_velocity("turn") == 0 ? this->a_data.angle : this->get_velocity("turn"));

		} else if (angular_point >= 0 && angular_point < this->a_data.p1) {

			result = this->a_data.acceleration * angular_point + this->a_data.initial_value;
			printf("a1\n");
			return result * this->sign(this->get_velocity("turn") == 0 ? this->a_data.angle : this->get_velocity("turn"));

		} else {

			this->stop_move("turn");

		}
	}

	return result;
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
	this->odom_data.linear_speed = odom->twist.twist.linear.x;
	this->odom_data.angular_speed = odom->twist.twist.angular.z;

}

#endif