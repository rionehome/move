#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

using namespace std;

ros::Publisher move_pub;

int main(int argc, char **argv) {

	ros::init(argc, argv, "target_move");

	ros::NodeHandle n;

	//std_msgs::Int32MultiArray pub_array;

	//ros::Subscriber sub = n.subscribe("target_move", 1000, move_info);

	move_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

	ros::spin();

	return 0;

}