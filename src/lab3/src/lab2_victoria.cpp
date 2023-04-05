#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <bits/stdc++.h>
#include <math.h>

#define pi acos(-1)
#define www 0.9
#define kp 0.15 * www
#define ka 0.4 * www
#define kb -0.075 * www



float goal[3];
float pos[5];
float pab[3];
float vw[2];
std_msgs::Bool goal_set;
float e[3];
bool pubbbbb = 0;

void cal_rad(float &tmp) {
	while (tmp > pi) {
		tmp -= 2*pi;
	}
	while (tmp < -pi) {
		tmp += 2*pi;
	}
}

float cal_rad2(float tmp) {
	
	while (tmp > pi) {
		tmp -= 2*pi;
	}
	while (tmp < -pi) {
		tmp += 2*pi;
	}
	return tmp;
}

void sub2_callback(const geometry_msgs::PoseStamped &msg) {
	// save goal

	goal_set.data = true;
	pubbbbb = 1;
	// pub2.publish(goal_set);

	goal[0]= msg.pose.position.x;
	goal[1]= msg.pose.position.y;

	float w = msg.pose.orientation.w;
	float z = msg.pose.orientation.z;
	float x = msg.pose.orientation.x;
	float y = msg.pose.orientation.y;
	goal[2] = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y));
	ROS_INFO_STREAM("GOAL : " << goal[0] << " " << goal[1] << " " << goal[2]);
	
}

void sub_callback(const geometry_msgs::Twist &msg) {
	// access current position
	pos[0] = msg.linear.x;
	pos[1] = msg.linear.y;
	pos[2] = cal_rad2(msg.angular.z);
}

void cal_pab() {
	float tmp;

	for (int i = 0; i < 3; i++) {
		e[i] = goal[i] - pos[i];
	}

	if (abs(e[0]) < 0.2 && abs(e[1]) < 0.2 && abs(e[2]) < 0.5) {
		goal_set.data = false;
		pubbbbb = 1;

		ROS_INFO_STREAM(" ---DOW DIAN--- ");
		return;
	}

	tmp = atan2(e[1], e[0]) - pos[2];
	cal_rad(tmp);
	cal_rad(e[2]);

	pab[0] = sqrt(e[0] * e[0] + e[1] * e[1]) - 0.01;
	pab[1] = tmp;
	pab[2] = e[2];
}

int main(int argc, char **argv) {
	goal_set.data = 0;
	pubbbbb = 1;
	ros::init(argc, argv, "pab_control");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Publisher pub2 = nh.advertise<std_msgs::Bool>("/tracking", 1000);
	
	ros::Subscriber sub = nh.subscribe("/robot_pose", 1000, sub_callback);
	ros::Subscriber sub2 = nh.subscribe("/goal_pos", 1000, sub2_callback);

	ros::Rate rate(100);
	pub2.publish(goal_set);

	while (ros::ok()) {
		geometry_msgs::Twist vel;

		if (goal_set.data) {
	        cal_pab();
			vw[0] = kp * pab[0];
            vw[1] = ka * pab[1] + kb * pab[2];
	        vel.linear.x = vw[0];
		    vel.angular.z = vw[1];
//			ROS_INFO_STREAM("linear veloctiy: " << vw[0] << " , angular: " << vw[1]);
		}
		else {
			vel.linear.x = 0;
			vel.angular.z = 0;
		}
		pub.publish(vel);
		if (pubbbbb) {
			pub2.publish(goal_set);
			pubbbbb = 0;
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
