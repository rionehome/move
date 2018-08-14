#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <time.h>


typedef struct {

	double x;
	double y;
	double z;
	double angular_x;
	double angular_y;
	double angular_z;
	double angular_w;

} Odometry;

using namespace std;

ros::Publisher pub;
Odometry odominfo;

double goal = 0;
double distance_info = 0.0;
bool flag = false;
double input_info;

void publish_msgs(const ros::Publisher& pub, double x, double az) {

	geometry_msgs::Twist twist;

	twist.linear.x = x;
	twist.angular.z = az;

	pub.publish(twist);

}

void calc_move(const nav_msgs::Odometry::ConstPtr& odom) {

	//if (flag == true) {

	double x = odom->pose.pose.position.x;

	//printf("%f\n", x );

	if (distance_info == 0) {
		goal = x + input_info;
	}

	distance_info =  goal - x;

	printf("%f\n", distance_info);

	if (distance_info > 0 && flag) {

		publish_msgs(pub, 0.1, 0);

	} else if (distance_info <= 0 && flag) {

		flag = false;
		distance_info = 0;

	}

}

void odometry(const nav_msgs::Odometry::ConstPtr& odom) {

	//printf("%f\n", odom->pose.pose.position.x );

	//printf("%f\n", odom->pose.pose.position.x);
	/*
		odominfo.x = odom->pose.pose.position.x;
		odominfo.y = odom->pose.pose.position.y;
		odominfo.z = odom->pose.pose.position.z;
		odominfo.angular_x = odom->pose.pose.orientation.x;
		odominfo.angular_y = odom->pose.pose.orientation.y;
		odominfo.angular_z = odom->pose.pose.orientation.z;
		odominfo.angular_w = odom->pose.pose.orientation.w;

		x = odom->pose.pose.position.x;
	*/
	calc_move(odom);

}

void test_move(const std_msgs::Float64::ConstPtr& input) {

	flag = true;
	input_info = input->data;

}

int main(int argc, char **argv) {

	geometry_msgs::Twist twist;

	ros::init(argc, argv, "test_move");

	double start = clock();
	double step = 0;

	ros::NodeHandle n;
	int deb;

	//ros::Subscriber sub = n.subscribe("/odom", 1000, odometry);
	//ros::Subscriber move = n.subscribe("/test_move", 1000, test_move);

	pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

	scanf("%d", &deb);

	while (1) {

		step = clock();

		printf("%f\n", step - start );

		if ((step - start) / CLOCKS_PER_SEC > 8.795546204) break;

		twist.angular.z = 1;
		pub.publish(twist);

	}

	twist.angular.z = 0;
	pub.publish(twist);

	return 0;

}