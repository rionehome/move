//
// Created by migly-home on 19/07/26.
//
#ifndef VELOCITY_H
#define VELOCITY_H
class Velocity
{
public:
    explicit Velocity(ros::NodeHandle *n);
    ~Velocity();
    void velocity_update();

private:
    ros::Subscriber velocity_sub;
    ros::Publisher twist_pub;
    double stack_linear = 0.0;
    double stack_angular = 0.0;
    double last_linear = 0.0;
    double last_linear_acceleration = 0.0;
    double last_angular = 0.0;
    double last_angular_acceleration = 0.0;

    void callbackVelocity(const move::Velocity_<std::allocator<void>>::ConstPtr &msg);
    void publishTwist(double liner_x, double angular_z);
    double toRadian(double angle)
    { return (angle * M_PI) / 180; }
};


#endif //VELOCITY_H
