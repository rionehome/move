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

	Odometry odom_data;

	void odometry(const nav_msgs::Odometry::ConstPtr &odom);
	void setOdom(string element);
	double getAmount(string element);

	double move_amount;

public:
	MovementAmountDesignation();
	~MovementAmountDesignation();

	void updata();
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
	this->signal = n.advertise<std_msgs::Int32 >("/move/signal", 1000);

}


MovementAmountDesignation::~MovementAmountDesignation() {

	printf("shutdown class of 'MovementAmountDesignation'\n");
}

void MovementAmountDesignation::updata() {

	ros::spinOnce();

}

void MovementAmountDesignation::setOdom(string element) {

	if (element == "straight") {

	}

	if (element == "turn") {

	}

}

double MovementAmountDesignation::getAmount(string element) {

	if (element == "straight") {

	}

	if (element == "turn") {

	}

	return -1;
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

int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementAmount");

	MovementAmountDesignation move;

	ros::spin();

	return 0;

}