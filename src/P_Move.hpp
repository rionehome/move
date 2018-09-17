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

class P_Move {
public:
	P_Move();
	~P_Move();
	void update() {ros::spinOnce();}

	double getPosition(string element);
	double getVelocity(string element);
	double getDistance(double x0, double y0);
	double getAngle(double a0);
	void setOdometry(const nav_msgs::Odometry::ConstPtr &odom);

	double toQuaternion_ang(double w, double z) {
		return abs((z > 0 ? 1 : 360) - this->toAngle(acos(w) * 2));
	}
	double toQuaternion_rad(double w, double z) {
		return acos(this->odom_data.angular_w) * (this->odom_data.angular_z > 0 ? 1 : -1) * 2;
	}

	double toAngle(double rad) {return rad * 180 / M_PI;}
	double toRadian(double angle) {return (angle * M_PI) / 180;}
	double sign(double A) {return  A == 0 ? 0 : A / abs(A);}

	double functionStraight(double k, double target);
	double functionTurn(double k, double target);

	void pubTwist(const ros::Publisher& pub, double v, double a);
	void pubSignal(const ros::Publisher& pub, int s);

private:
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

	Odometry odom_data;
};

P_Move::P_Move() {
	printf("Start class of 'P_Move'\n");
}

P_Move::~P_Move() {
	printf("Shutdown class of 'P_Move'\n");
}

//現在の座標を取得
double P_Move::getPosition(string element) {

	if (element == "x") return this->odom_data.x;
	if (element == "y") return this->odom_data.y;
	if (element == "angle") return this->toQuaternion_ang(this->odom_data.angular_w, this->odom_data.angular_z);

	return 1;
}

//現在の速度を取得
double P_Move::getVelocity(string element) {

	if (element == "straight") {
		return abs(this->odom_data.linear_speed) > 0.01 ? this->odom_data.linear_speed : 0;
	}

	if (element == "turn") {
		return abs(this->odom_data.angular_speed) > 0.01 ? this->odom_data.angular_speed : 0;
	}

	return 1;
}

//距離の取得
double P_Move::getDistance(double x0, double y0) {

	return hypot(this->getPosition("x") - x0, this->getPosition("y") - y0);
}

//角度の取得
double P_Move::getAngle(double a0) {

	

	return 0;
}

//速度のP制御
double P_Move::functionStraight(double k, double targetV) {

	double vel;
	//velocity += k * (targetV - x);

	vel = k * (targetV - this->getVelocity("straight")) + this->getVelocity("straight");
	printf("vel %f\n", vel );
	return abs(vel) < 0.01 ? 0 : vel;
}

//角度のP制御
double P_Move::functionTurn(double k, double targetA) {

	double ang;

	//angular += k * (targetA - a);

	ang = k * (targetA - this->getVelocity("turn")) + this->getVelocity("turn");
	printf("ang %f\n", ang);
	return abs(ang) < 0.1 ? 0 : ang;
}

//Twist送信
void P_Move::pubTwist(const ros::Publisher &pub, double v, double a) {

	geometry_msgs::Twist twist;

	twist.linear.x = v;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = a;

	pub.publish(twist);
}

//シグナル送信
void P_Move::pubSignal(const ros::Publisher &pub, int s) {

	std_msgs::Int32 signal;

	signal.data = s;

	pub.publish(signal);
}

//オドメトリ登録
void P_Move::setOdometry(const nav_msgs::Odometry::ConstPtr &odom) {

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