#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>

using namespace std;

class MovementPoint {
private:
	ros::NodeHandle n;
	ros::Subscriber odom;
	ros::Subscriber point;
	ros::Subscriber move_signal;
	ros::Publisher amount;

	int signal_flag = 0;

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
		signal_flag = msg->data;
	}

public:
	MovementPoint();
	~MovementPoint();

	void calc(const std_msgs::Float64MultiArray::ConstPtr &msg);
	void updata() {ros::spinOnce();}
	double angle_to_quaternion(double w, double z) {
		return abs((z > 0 ? 1 : 360) - this->angle_to_rad(acos(w) * 2));
	}
	double angle_to_rad(double rad) {return rad * 180 / M_PI;}

};

MovementPoint::MovementPoint() {

	printf("start class of 'MovementPoint'\n");

	this->odom = n.subscribe("/odom", 1000, &MovementPoint::odometry, this);
	this->point = n.subscribe("/move/point", 1000, &MovementPoint::callback, this);
	this->move_signal = n.subscribe("/move/signal", 1000, &MovementPoint::signal, this);
	this->amount = n.advertise<std_msgs::Float64MultiArray>("/move/amount", 1000);

}

MovementPoint::~MovementPoint() {

	printf("shutdown class of 'MovementPoint'\n");
}

void MovementPoint::pub_msg(double distance, double angle) {

	std_msgs::Float64MultiArray msg;

	msg.data.clear();
	msg.data.push_back(distance);
	msg.data.push_back(0.3);
	msg.data.push_back(angle);
	msg.data.push_back(1.5);

	amount.publish(msg);

}

void MovementPoint::odometry(const nav_msgs::Odometry::ConstPtr &odom) {

	this->odom_data.x = odom->pose.pose.position.x;
	this->odom_data.y = odom->pose.pose.position.y;
	this->odom_data.z = odom->pose.pose.position.z;
	this->odom_data.angular_x = odom->pose.pose.orientation.x;
	this->odom_data.angular_y = odom->pose.pose.orientation.y;
	this->odom_data.angular_z = odom->pose.pose.orientation.z;
	this->odom_data.angular_w = odom->pose.pose.orientation.w;

}

void MovementPoint::callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {

	this->calc(msg);

}

void MovementPoint::calc(const std_msgs::Float64MultiArray::ConstPtr &msg) {

	//[0]:x  [1]:y
	double x0 = this->odom_data.x;
	double y0 = this->odom_data.y;
	double bot_angle = this->angle_to_quaternion(this->odom_data.angular_w, this->odom_data.angular_z);
	double angle = 0;
	double result;

	if (x0 == msg->data[0] && y0 == msg->data[1]) {
		return;
	}

	//角度取得
	if (y0 == msg->data[1]) {

		angle = 90;

	} else {

		angle = angle_to_rad(atan2(y0 - msg->data[1], x0 - msg->data[0]));

	}

	result = 180 - (180 - angle) + (180 - bot_angle);

	this->pub_msg(0, result);

	while (ros::ok()) {
		this->updata();
		if (this->signal_flag == 1) {
			this->signal_flag = 0;
			break;
		} else if (this->signal_flag == -1) {
			printf("################Error############3\n");
			exit(1);
		}
	}

	this->pub_msg(hypot(x0 - msg->data[0], y0 - msg->data[1]), 0);

	while (ros::ok()) {
		this->updata();
		if (this->signal_flag == 1) {
			this->signal_flag = 0;
			break;
		} else if (this->signal_flag == -1) {
			printf("################Error############3\n");
			exit(1);
		}
	}

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementPoint");

	MovementPoint move;

	ros::spin();

	return 0;
}