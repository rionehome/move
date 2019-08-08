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
    double sensor_x = 0.0;
    double sensor_y = 0.0;
    double initial_x = 0.0;
    double initial_y = 0.0;
    double sensor_distance = 0.0;
    double sensor_angular = 0.0;
    double sensor_angle = 0.0;
    double target_distance = 0.0;
    double target_linear = 0.0;
    double target_angle = 0.0;
    double target_angler = 0.0;

    double integral_linear = 0.0;
    double integral_angular = 0.0;
    double diff_linear[2]{};
    double diff_angular[2]{};
    double diff_distance[2]{};
    bool move_flag = false;

    static double toAngle(double rad)
    { return rad * 180 / M_PI; }

    double toQuaternion_ang(double w, double z)
    {
        return std::abs((z > 0 ? 1 : 360) - toAngle(acos(w) * 2));
    }

    double advancedDistance(double x, double y)
    {
        return hypot(x - initial_x, y - initial_y);
    }

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
            this->target_angler = msg->angular_rate * MAX_ANGULAR;
            this->move_flag = true;
        }
        else {
            printf("angular_rateの値が-1~1の範囲外です。\n");
        }
        this->target_distance = msg->distance;
        this->target_angle = msg->angle;
        this->initial_x = this->sensor_x;
        this->initial_y = this->sensor_y;
    }

    void callbackOdometry(const nav_msgs::Odometry::ConstPtr &msg)
    {
        /*
         * オドメトリ情報を受け取る
         */
        this->sensor_linear = msg->twist.twist.linear.x;
        this->sensor_angular = msg->twist.twist.angular.z;
        this->sensor_x = msg->pose.pose.position.x;
        this->sensor_y = msg->pose.pose.position.y;
        this->sensor_distance = advancedDistance(msg->pose.pose.position.x, msg->pose.pose.position.y);
        this->sensor_angle = toQuaternion_ang(msg->pose.pose.orientation.w, msg->pose.pose.orientation.z);
    }
    void publishTwist(double liner_x, double angular_z);
    double distancePidControl(double Kp, double Ki, double Kd);
    double angularPidControl(double Kp, double Ki, double Kd);
};

#endif //AMOUNT_H
