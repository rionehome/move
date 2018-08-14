#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char **argv) {

	double i;

	ros::init(argc, argv, "test_send");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/move/distance", 1000);

	std_msgs::Float64MultiArray info;

	while (ros::ok()) {

		scanf("%lf", &i);

		info.data[0] = i;

		pub.publish(info);

	}

	return 0;

}