#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char **argv) {

	double a, b, c, d;

	ros::init(argc, argv, "test_send");

	ros::NodeHandle n;
	/*
		ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/move/amount", 1000);

		std_msgs::Float64MultiArray info;

		while (ros::ok()) {

			info.data.clear();

			scanf("%lf", &a);
			scanf("%lf", &c);

			b = 0.3;
			d = 1.5;

			info.data.push_back(a);
			info.data.push_back(b);
			info.data.push_back(c);
			info.data.push_back(d);

			pub.publish(info);

		}
		*/

	ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/move/point", 1000);

	std_msgs::Float64MultiArray info;

	info.data.clear();

	while (ros::ok()) {

		scanf("%lf", &a);
		scanf("%lf", &b);

		info.data.push_back(a);
		info.data.push_back(b);

		pub.publish(info);
		printf("send\n");

	}

	return 0;

}