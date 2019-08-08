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
    ros::Publisher twist_pub;
    ros::Publisher velocity_pub;
    double sensor_x = 0.0;
    double sensor_y = 0.0;
    double initial_x = 0.0;
    double initial_y = 0.0;
    double sensor_distance = 0.0;
    double sensor_angle = 0.0;
    double target_distance = 0.0;
    double target_angle = 0.0;

    double integral_linear = 0.0;
    double integral_angular = 0.0;
    double diff_linear[2]{};
    double diff_angular[2]{};
    bool move_flag = false;

    static double toAngle(double rad)
    { return rad * 180 / M_PI; }

    static double toQuaternion_ang(double w, double z)
    {
        return std::abs((z > 0 ? 1 : 360) - toAngle(acos(w) * 2));
    }

    double advancedDistance(double x, double y)
    {
        return hypot(x - initial_x, y - initial_y) * (std::signbit(x - initial_x) ? -1 : 1);
    }

    void callbackWheeDrop(const kobuki_msgs::WheelDropEvent::ConstPtr &msg)
    {
        /*
         * タイヤの接地状態を受け取り
         */
        if (msg->state == 1) {
            this->move_flag = false;
            //this->publishTwist(0.0, 0.0);
            printf("タイヤが浮きました\n");
        }
    }

    void callbackAmount(const move::Amount::ConstPtr &msg)
    {
        /*
         * 直進距離と角度の情報を受け取り.
         */
        this->move_flag = true;
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
        this->sensor_x = msg->pose.pose.position.x;
        this->sensor_y = msg->pose.pose.position.y;
        this->sensor_distance = advancedDistance(msg->pose.pose.position.x, msg->pose.pose.position.y);
        this->sensor_angle = toQuaternion_ang(msg->pose.pose.orientation.w, msg->pose.pose.orientation.z);
    }
    double distancePidControl(double Kp, double Ki, double Kd);
    double angularPidControl(double Kp, double Ki, double Kd);
};

#endif //AMOUNT_H
