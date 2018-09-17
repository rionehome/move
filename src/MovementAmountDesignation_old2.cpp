#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include "Move.hpp"

#define ACCELERATION 2
#define ANGLE_ACCELERATION 0.03

using namespace std;

Move m;

class MovementAmountDesignation {
public:
	MovementAmountDesignation();
	~MovementAmountDesignation();

	ros::NodeHandle n;
	ros::Subscriber odom;
	ros::Subscriber amout_move;
	ros::Publisher pub_move;
	ros::Publisher signal;

	void calc(const std_msgs::Float64MultiArray::ConstPtr &msgs);
	void updata() {m.updata();}

private:

	void set_odom(const nav_msgs::Odometry::ConstPtr &odom) {m.set_odometry(odom);}

};


///////////////////////////////////////////////////////////////////////////////////

MovementAmountDesignation::MovementAmountDesignation() { //初期化

	printf("start class of 'MovementAmountDesignation'\n");

	this->odom = n.subscribe("/odom", 1000, &MovementAmountDesignation::set_odom, this);
	this->amout_move = n.subscribe("/move/amount", 1000, &MovementAmountDesignation::calc, this);
	this->pub_move = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	this->signal = n.advertise<std_msgs::Int32 >("/move/signal", 1000);

}


MovementAmountDesignation::~MovementAmountDesignation() { //終了

	printf("shutdown class of 'MovementAmountDesignation'\n");
}


//移動量受信
void MovementAmountDesignation::calc(const std_msgs::Float64MultiArray::ConstPtr &msgs) {

	if (m.get_velocity("straight") == 0 && m.get_velocity("turn") == 0) { //静止の場合

		printf("静止\n");
		m.reset_progress("straight");
		m.reset_progress("turn");

		m.create_straight(msgs->data[1], msgs->data[0], ACCELERATION, 0.1); //速度プロファイル作成
		m.create_turn(msgs->data[3], msgs->data[2], ANGLE_ACCELERATION, 0.8, 0); //速度プロファイル作成

	} else {

		printf("動作中\n");
		m.reset_progress("straight"); //初期位置初期化
		m.reset_progress("turn");

		//同方向
		if (m.sign(m.get_velocity("straight")) == m.sign(msgs->data[0]))
			m.create_straight(msgs->data[1], msgs->data[0], ACCELERATION, m.get_velocity("straight")); //速度プロファイル作成

		if (m.sign(m.get_velocity("turn")) == m.sign(msgs->data[2]))
			m.create_turn(msgs->data[3], msgs->data[2], ANGLE_ACCELERATION, m.get_velocity("turn"), 0); //速度プロファイル作成

		//停止
		if (msgs->data[0] == 0)
			m.create_straight(m.get_velocity("straight"), (m.get_velocity("straight") / ACCELERATION) * m.sign(m.get_velocity("straight")), ACCELERATION, m.get_velocity("straight")); //速度プロファイル作成

		if (msgs->data[2] == 0)
			m.create_turn(m.get_velocity("turn"), (m.get_velocity("turn") / ANGLE_ACCELERATION) * m.sign(m.get_velocity("turn")) , ANGLE_ACCELERATION, m.get_velocity("turn"), 0); //速度プロファイル作成

		//逆方向
	

	}

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {

	ros::init(argc, argv, "MovementAmount");

	MovementAmountDesignation amount;

	ros::Rate loop_rate(8);

	while (ros::ok()) {

		amount.updata();

		m.add_progress("straight", m.get_variate("straight"));

		m.add_progress("turn", m.get_variate("turn"));

		printf("progress %f\n", m.get_progress("turn") );

		m.pub_twist(amount.pub_move, m.straight(m.get_progress("straight")), m.turn(m.get_progress("turn"))); //送信

		loop_rate.sleep();

	}

	return 0;

}