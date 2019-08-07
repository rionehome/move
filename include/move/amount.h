//
// Created by migly-home on 19/07/26.
//
#ifndef AMOUNT_H
#define AMOUNT_H
#define MAX_LINEAR 0.7 // m/s
#define MAX_ANGULAR 1.9 // rad

class Amount
{
public:
    explicit Amount(ros::NodeHandle *n);
    ~Amount();
    void amount_update();

private:
    ros::Subscriber amount_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber wheel_drop_sub;
    ros::Publisher twist_pub;
    double stack_linear = 0.0;
    double stack_angular = 0.0;
    double sensor_linear = 0.0;
    double sensor_angular = 0.0;
    double target_linear = 0.0;
    double target_angular = 0.0;
    double integral_linear = 0.0;
    double integral_angular = 0.0;
    double diff_linear[2]{};
    double diff_angular[2]{};
    bool move_flag = false;

    void callbackWheeDrop(const kobuki_msgs::WheelDropEvent::ConstPtr &msg)
    {
        /*
         * タイヤの接地状態を受け取り
         */
        if (msg->state == 1) {
            this->move_flag = false;
            this->publishTwist(0.0, 0.0);
            printf("タイヤが浮きました\n");
        }
    }

    void callbackAmount(const move::Amount::ConstPtr &msg)
    {
        /*
         * 直進、回転の加速度と速度の情報を受け取り.
         * linear:0.7m/sが最大.
         * angular:110deg/sが最大.
         */
        if (std::abs(msg->linear_rate) <= 1.0) {
            this->target_linear = msg->linear_rate * MAX_LINEAR;
            this->move_flag = true;
        }
        else {
            printf("linear_rateの値が-1~1の範囲外です。\n");
        }
        if (std::abs(msg->angular_rate) <= 1.0) {
            this->target_angular = msg->angular_rate * MAX_ANGULAR;
            this->move_flag = true;
        }
        else {
            printf("angular_rateの値が-1~1の範囲外です。\n");
        }
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

#endif //AMOUNT_H
