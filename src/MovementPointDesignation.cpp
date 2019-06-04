#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>

using namespace std;

class MovementPointDesignation {
private:
	ros::NodeHandle n;
	ros::Subscriber odom;
	ros::Subscriber point;
	ros::Subscriber move_signal;
	ros::Publisher amount;
	ros::Publisher point_signal;

	int amount_signal_flag = 0;
	int point_signal_flag = 0;


	typedef struct {
		double x;
		double y;
		double z;
		double angular_x;
		double angular_y;
		double angular_z;
		double angular_w;
	} Odometry;

	Odometry odom_data;

	void odometry(const nav_msgs::Odometry::ConstPtr &odom);
	void callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
	void pub_msg(double distance, double angle);
	void signal(const std_msgs::Int32::ConstPtr &msg) {
		amount_signal_flag = msg->data;
	}

public:
	MovementPointDesignation();
	~MovementPointDesignation();

	void calc(const std_msgs::Float64MultiArray::ConstPtr &msg);
	double calcAngle_point(double x, double y);
	void pubSignal(const ros::Publisher& pub, int s);
	void updata() {ros::spinOnce();}
	double angle_to_quaternion(double w, double z) {
		return abs((z > 0 ? 1 : 360) - this->angle_to_rad(acos(w) * 2));
	}
	double angle_to_rad(double rad) {return rad * 180 / M_PI;}

};

MovementPointDesignation::MovementPointDesignation() {
	printf("start class of 'MovementPointDesignation'\n");
	this->odom = n.subscribe("/odom", 1000, &MovementPointDesignation::odometry, this);
	this->point = n.subscribe("/move/point", 1000, &MovementPointDesignation::callback, this);
	this->move_signal = n.subscribe("/move/amount/signal", 1000, &MovementPointDesignation::signal, this);
	this->amount = n.advertise<std_msgs::Float64MultiArray>("/move/amount", 1000);
	this->point_signal = n.advertise<std_msgs::Int32>("/move/point/signal", 1000);
}

MovementPointDesignation::~MovementPointDesignation() {

	printf("shutdown class of 'MovementPointDesignation'\n");
}

void MovementPointDesignation::pub_msg(double distance, double angle) {

	std_msgs::Float64MultiArray msg;

	msg.data.clear();
	msg.data.push_back(distance);
	msg.data.push_back(0.3);
	msg.data.push_back(angle);
	msg.data.push_back(1.5);

	amount.publish(msg);

}

//シグナル送信
void MovementPointDesignation::pubSignal(const ros::Publisher &pub, int s) {

	std_msgs::Int32 signal;

	signal.data = s;

	pub.publish(signal);
}

double MovementPointDesignation::calcAngle_point(double x, double y) {
	double result;
	if (x == 0) {
		if (y < 0) {
			return 270.0;
		} else if (y > 0) {
			return 90.0;
		} else {
			return 0.0;
		}
	}
	result = atan(y / x);
	printf("%f\n", x);
	printf("%f\n", y);
	if (x < 0) result = M_PI + result;
	if (result < 0) result = 2 * M_PI + result;
	return angle_to_rad(result);
}

void MovementPointDesignation::odometry(const nav_msgs::Odometry::ConstPtr &odom) {
	this->odom_data.x = odom->pose.pose.position.x;
	this->odom_data.y = odom->pose.pose.position.y;
	this->odom_data.z = odom->pose.pose.position.z;
	this->odom_data.angular_x = odom->pose.pose.orientation.x;
	this->odom_data.angular_y = odom->pose.pose.orientation.y;
	this->odom_data.angular_z = odom->pose.pose.orientation.z;
	this->odom_data.angular_w = odom->pose.pose.orientation.w;
}

void MovementPointDesignation::callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
	this->calc(msg);
}

void MovementPointDesignation::calc(const std_msgs::Float64MultiArray::ConstPtr &msg) {

	//[0]:x  [1]:y
	double x0 = this->odom_data.x;
	double y0 = this->odom_data.y;
	double bot_angle = this->angle_to_quaternion(this->odom_data.angular_w, this->odom_data.angular_z);
	double result;
	double course_angle;

	if (x0 == msg->data[0] && y0 == msg->data[1]) {
		return;
	}

	printf("x0 %f\n", x0 );
	printf("y0 %f\n", y0 );


	course_angle = this->calcAngle_point(msg->data[0] - x0, msg->data[1] - y0);
	printf("course_angle %f\n", course_angle);
	printf("bot_angle %f\n", bot_angle);
	result = course_angle - bot_angle;

	if (abs(result) > 180) result = (360 - abs(result)) * (result > 0 ? -1 : 1);

	printf("%f\n", result);

	this->pub_msg(0, result);

	this->amount_signal_flag = 1;

	//point_signal送信
	pubSignal(point_signal, 1);

	while (ros::ok()) {
		this->updata();
		if (this->amount_signal_flag == 0) {
			//this->amount_signal_flag = 1;
			break;
		} else if (this->amount_signal_flag == -1) {
			printf("################Error#############\n");
			exit(1);
		}
	}

	this->pub_msg(hypot(x0 - msg->data[0], y0 - msg->data[1]), 0);

	this->amount_signal_flag = 1;

	while (ros::ok()) {
		this->updata();
		if (this->amount_signal_flag == 0) {
			//this->amount_signal_flag = 1;
			printf("finish\n");
			//point_signal終了シグナル送信
			pubSignal(point_signal, 0);
			break;
		} else if (this->amount_signal_flag == -1) {
			printf("################Error#############\n");
			exit(1);
		}
	}

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementPointDesignation");

	MovementPointDesignation move;

	ros::spin();

	return 0;
}