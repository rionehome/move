//
// Created by migly-home on 19/07/26.
//
#include "ros/ros.h"

#ifndef VELOCITY_H
#define VELOCITY_H
class Velocity
{
public:
    explicit Velocity(ros::NodeHandle *n);
    ~Velocity();

    ros::Subscriber odom;
    ros::Subscriber velocity;
    ros::Publisher twist;

    void setOdometry(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &msg);
private:
    typedef struct
    {
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
    Odometry odometry;
};


#endif //VELOCITY_H
