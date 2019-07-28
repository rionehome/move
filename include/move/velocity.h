//
// Created by migly-home on 19/07/26.
//
#ifndef VELOCITY_H
#define VELOCITY_H
#define MAX_LINEAR 0.7 // m/s
#define MAX_ANGULAR 1.9 // rad

class Velocity
{
public:
    explicit Velocity(ros::NodeHandle *n);
    ~Velocity();
    void velocity_update();

private:
    ros::Subscriber velocity_sub;
    ros::Subscriber odometry_sub;
    ros::Publisher twist_pub;
    double stack_angular = 0.0;
    double sensor_linear = 0.0;
    double sensor_angular = 0.0;
    double target_linear = 0.0;
    double target_angular = 0.0;
    double integral_linear = 0.0;
    double integral_angular = 0.0;
    double diff_linear[2]{};
    double diff_angular[2]{};

    void callbackVelocity(const move::Velocity::ConstPtr &msg)
    {
        /*
         * 直進、回転の加速度と速度の情報を受け取り.
         * linear:0.7m/sが最大.
         * angular:110deg/sが最大.
         */
        if (std::abs(msg->linear_rate) <= 1.0)
            this->target_linear = msg->linear_rate * MAX_LINEAR;
        if (std::abs(msg->angular_rate) <= 1.0)
            this->target_angular = msg->angular_rate * MAX_ANGULAR;
    }

    void callbackOdometry(const nav_msgs::Odometry::ConstPtr &msg)
    {
        /*
         * オドメトリ情報を受け取る
         */
        this->sensor_linear = msg->twist.twist.linear.x;
        this->sensor_angular = msg->twist.twist.angular.z;
    }
    void publishTwist(double liner_x, double angular_z);
    double linearPidControl(double Kp, double Ki, double Kd);
    double angularPidControl(double Kp, double Ki, double Kd);
};


#endif //VELOCITY_H
