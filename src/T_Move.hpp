#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>

#ifndef   T_MOVE_HPP
#define   T_MOVE_HPP

using namespace std;

class T_Move {
public:
	T_Move();
	~T_Move();

	void init();

	void update();

	double getPosition(string element);
	double getVelocity(string element);
	void resetAmount();
	double getAmount(string element);
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

	//Amount
	double exeDistance(double targetAmount, double max_v, double point);
	double exeAngle(double targetAmount, double max_v, double point);

	//Velocity
	double calcVelocityStraight(double d, double target);
	double calcVelocityTurn(double d, double target);

	void pubTwist(const ros::Publisher& pub, double v, double a);
	void pubSignal(const ros::Publisher& pub, int s);
	void pubVelocity(const ros::Publisher& pub, double v, double v_a, double a, double a_a);

private:

	double stackStraightVelocity = 0;
	double stackTurnVelocity = 0;

	typedef struct {
		double data[2];
		double stack = 0;
	} Amount;

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
		bool enable = false;
	} Odometry;

	Odometry odom_data;
	Amount straight_data;
	Amount turn_data;

	void updateAmount();
};

T_Move::T_Move() {
	printf("Start class of 'T_Move'\n");
}

T_Move::~T_Move() {
	printf("Shutdown class of 'T_Move'\n");
}

void T_Move::init() {

	ros::spinOnce();
	while (!this->odom_data.enable) ros::spinOnce(); //オドメトリ情報取得まで待機
}

void T_Move::update() {

	ros::spinOnce();
	this->updateAmount();
}

//現在の座標を取得
double T_Move::getPosition(string element) {

	if (element == "enable") return this->odom_data.enable;
	if (element == "x") return this->odom_data.x;
	if (element == "y") return this->odom_data.y;
	if (element == "angle") return this->toQuaternion_ang(this->odom_data.angular_w, this->odom_data.angular_z);

	return -1;
}

//現在の速度を取得
double T_Move::getVelocity(string element) {

	if (element == "straight") {
		return abs(this->odom_data.linear_speed) > 0.01 ? this->odom_data.linear_speed : 0;
	}

	if (element == "turn") {
		return abs(this->odom_data.angular_speed) > 0.01 ? this->odom_data.angular_speed : 0;
	}

	return -1;
}

//移動距離の設定
void T_Move::resetAmount() {

	//straight
	this->straight_data.stack = 0;

	//angle
	this->turn_data.stack = 0;

	this->straight_data.data[0] = this->getPosition("x");
	this->straight_data.data[1] = this->getPosition("y");
	this->turn_data.data[1] = this->getPosition("angle");
	this->turn_data.data[0] = this->getPosition("angle");
}

//移動距離の更新
void T_Move::updateAmount() {

	//straight_update
	this->straight_data.stack += hypot((this->getPosition("x") - this->straight_data.data[0]), (this->getPosition("y") - this->straight_data.data[1]));

	//anglar_update
	double a_delta;

	a_delta = this->turn_data.data[0] - this->turn_data.data[1];

	if (a_delta < 0 && abs(a_delta) > 180) a_delta = this->turn_data.data[0] + (360 - this->turn_data.data[1]);
	if (a_delta > 0 && abs(a_delta) > 180) a_delta = -1 * (this->turn_data.data[1] + (360 - this->turn_data.data[0]));

	this->turn_data.stack += a_delta;

	this->turn_data.data[1] = this->turn_data.data[0];
	this->turn_data.data[0] = this->getPosition("angle");
}

//移動距離の取得
double T_Move::getAmount(string element) {

	if (element == "straight") return this->straight_data.stack;

	if (element == "turn") return this->turn_data.stack;

	return -1;
}

double T_Move::exeDistance(double targetAmount, double max_v, double point) {

	if (targetAmount == 0) return 0;

	if (abs(targetAmount) < point) return 0;
	printf("p2\n");
	if (abs(targetAmount) / 2 > point) return this->calcVelocityStraight(0.3, this->sign(targetAmount) * max_v); //移動距離の半分まで
	printf("p3\n");
	return this->calcVelocityStraight((point / targetAmount) / 10, 0);
}

double T_Move::exeAngle(double targetAmount, double max_v, double point) {


	return 0;
}

//速度のT制御
double T_Move::calcVelocityStraight(double d, double targetV) {

	if (targetV - this->stackStraightVelocity > 0) {

		this->stackStraightVelocity += d;

		if (this->stackStraightVelocity > targetV) this->stackStraightVelocity = targetV;

	} else {

		this->stackStraightVelocity -= d;

		if (this->stackStraightVelocity < targetV) this->stackStraightVelocity = targetV;

	}

	if (abs(this->stackStraightVelocity) < 0.01) this->stackStraightVelocity = 0;

	return this->stackStraightVelocity;
}

//角度のT制御
double T_Move::calcVelocityTurn(double d, double targetA) {

	if (targetA - this->stackTurnVelocity >= 0) {

		this->stackTurnVelocity += d;

		if (this->stackTurnVelocity > targetA) this->stackTurnVelocity = targetA;

	} else {

		this->stackTurnVelocity -= d;

		if (this->stackTurnVelocity < targetA) this->stackTurnVelocity = targetA;

	}

	if (abs(this->stackTurnVelocity) < 0.01) this->stackTurnVelocity = 0;

	return this->stackTurnVelocity;
}

//Twist送信
void T_Move::pubTwist(const ros::Publisher &pub, double v, double a) {

	geometry_msgs::Twist twist;

	twist.linear.x = v;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = a;

	pub.publish(twist);
}

//速度送信
void T_Move::pubVelocity(const ros::Publisher &pub, double v, double v_a, double a, double a_a) {

	std_msgs::Float64MultiArray info;

	info.data.clear();

	info.data.push_back(v);
	info.data.push_back(v_a);
	info.data.push_back(a);
	info.data.push_back(a_a);

	pub.publish(info);
}

//シグナル送信
void T_Move::pubSignal(const ros::Publisher &pub, int s) {

	std_msgs::Int32 signal;

	signal.data = s;

	pub.publish(signal);
}

//オドメトリ登録
void T_Move::setOdometry(const nav_msgs::Odometry::ConstPtr &odom) {

	this->odom_data.x = odom->pose.pose.position.x;
	this->odom_data.y = odom->pose.pose.position.y;
	this->odom_data.z = odom->pose.pose.position.z;
	this->odom_data.angular_x = odom->pose.pose.orientation.x;
	this->odom_data.angular_y = odom->pose.pose.orientation.y;
	this->odom_data.angular_z = odom->pose.pose.orientation.z;
	this->odom_data.angular_w = odom->pose.pose.orientation.w;
	this->odom_data.linear_speed = odom->twist.twist.linear.x;
	this->odom_data.angular_speed = odom->twist.twist.angular.z;
	this->odom_data.enable = true;
}

#endif