#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

using namespace std;

ros::Publisher move_pub;

typedef struct Odometry
{

    double x;
    double y;
    double z;
    double angular_x;
    double angular_y;
    double angular_z;

} odometry;

void publish_msgs(const ros::Publisher &pub, double x, double az)
{

    geometry_msgs::Twist twist;

    twist.linear.x = x;
    twist.angular.z = az;

    pub.publish(twist);

}

void move_linear(const std_msgs::Float64::ConstPtr &linear)
{


    return;

}

void move_angular(const std_msgs::Float64::ConstPtr &angular)
{

}

void odom(const nav_msgs::Odometry::ConstPtr &odom)
{

    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "amount_move");

    ros::NodeHandle n;

    ros::Subscriber sub_linear = n.subscribe("amount_move_linear", 1000, move_linear);
    ros::Subscriber sub_angular = n.subscribe("amount_move_angular", 1000, move_angular);
    //ros::Subscriber odom = n.subscribe("/mobile_base/commands/velocity", 1000, odom); //?

    move_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

    ros::spin();

    return 0;

}