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
    typedef actionlib::SimpleActionServer<move::AmountAction> Server;

    explicit Amount(ros::NodeHandle *n);
    ~Amount();
    void amount_update();
    void rosUpdate();

private:
    ros::Subscriber odometry_sub;
    ros::Publisher twist_pub;
    ros::Publisher velocity_pub;
    move::AmountGoalConstPtr current_goal;
    Server *server;

    double sensor_x = 0.0;
    double sensor_y = 0.0;
    double sensor_q_z = 0.0;
    double sensor_q_w = 0.0;
    double initial_x = 0.0;
    double initial_y = 0.0;
    double last_q_w = 0.0;
    double last_q_z = 0.0;
    double sensor_distance = 0.0;
    double sensor_angle = 0.0;
    double target_linear_rate = 0.0;
    double target_distance = 0.0;
    double target_angular_rate = 0.0;
    double target_angle = 0.0;
    double integral_linear = 0.0;
    double integral_angular = 0.0;
    double diff_linear[2]{};
    double diff_angular[2]{};
    bool move_flag = false;

    static double toAngle(double rad)
    { return rad * 180 / M_PI; }

    double toQuaternion_ang(double w, double z)
    {
        return std::abs((z > 0 ? 1 : 360) - toAngle(acos(w) * 2));
    }

    double historyDistance(double x, double y)
    {
        return hypot(x - initial_x, y - initial_y) * (std::signbit(x - initial_x) ? -1 : 1);
    }

    double historyAngle(double angle)
    {
        //std::cout << angle - this->sensor_angle << '\n';
        double delta_angle = angle - this->toQuaternion_ang(this->last_q_w, this->last_q_z);
        this->last_q_w = this->sensor_q_w;
        this->last_q_z = this->sensor_q_z;
        if (std::abs(delta_angle) > 180)
            delta_angle = (360 - std::abs(delta_angle)) * (std::signbit(delta_angle) ? 1 : -1);
        return delta_angle;
    }

    void callbackOdometry(const nav_msgs::Odometry::ConstPtr &msg)
    {
        /*
         * オドメトリ情報を受け取る
         */
        this->sensor_x = msg->pose.pose.position.x;
        this->sensor_y = msg->pose.pose.position.y;
        this->sensor_q_w = msg->pose.pose.orientation.w;
        this->sensor_q_z = msg->pose.pose.orientation.z;
        this->sensor_distance = this->historyDistance(this->sensor_x, this->sensor_y);
        this->sensor_angle += this->historyAngle(this->toQuaternion_ang(this->sensor_q_w, this->sensor_q_z));
    }
    double distancePidControl(double Kp, double Ki, double Kd);
    double angularPidControl(double Kp, double Ki, double Kd);
};

#endif //AMOUNT_H
