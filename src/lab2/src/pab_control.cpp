#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <stdbool.h>

#define kp 3
#define ka 8 
#define kb -3
#define adj 0.05

#define pi M_PI

float n_x, n_y, n_theta, n_theta_pipi;
float g_x, g_y, g_theta, g_theta_pipi;
float e_x, e_y, e_theta;
float p, a, b;
int isgoal;


void pose_callback(const geometry_msgs::Twist &msg) {
	n_x = msg.linear.x;
	n_y = msg.linear.y;
	n_theta = msg.angular.z;
//	if(n_theta > pi)
//		n_theta_pipi = n_theta - 2*pi;
//	else if(n_theta < -pi)
//		n_theta_pipi = n_theta + 2*pi;
	while(n_theta > pi)
		n_theta -= 2*pi;
	while(n_theta < -pi)
		n_theta += 2*pi;
	//ROS_INFO("nx= %f, ny= %f, nth= %f", n_x, n_y, n_theta_pipi);
}

void goal_callback(const geometry_msgs::PoseStamped &msg) {
	g_x = msg.pose.position.x;
	g_y = msg.pose.position.y;
	g_theta = atan2(2*(msg.pose.orientation.w*msg.pose.orientation.z+msg.pose.orientation.x*msg.pose.orientation.y), 1-2*(msg.pose.orientation.z*msg.pose.orientation.z-msg.pose.orientation.y*msg.pose.orientation.y));
	ROS_INFO("gx= %f, gy= %f, gth= %f", g_x, g_y, g_theta);
}

bool isGoal(float n_x, float n_y, float n_theta_pipi, float g_x, float g_y, float g_theta){		
	//return (abs(g_x-n_x) < 0.1 && abs(g_y-n_y) < 0.1 && abs(g_theta-n_theta_pipi) < 1);
	if(abs(g_x-n_x) < 0.1 && abs(g_y-n_y) < 0.1 && abs(e_theta) < 1)
		isgoal=1;
	else	
		isgoal=0;
	return isgoal;
}

double pab_calculation(float n_x, float n_y, float n_theta, float n_theta_pipi, float g_x, float g_y, float g_theta){
	e_x = g_x - n_x;
	e_y = g_y - n_y;
	//e_theta = g_theta - n_theta_pipi;
	e_theta = g_theta - n_theta;
	p = sqrt(e_y*e_y+e_x*e_x);
	//a = -1*n_theta_pipi + atan2(e_y+0.0001, e_x+0.0001);
	a = -n_theta + atan2(e_y+0.0001, e_x+0.0001);
	b = e_theta;	
	while(a > pi)
		a -= 2*pi;
	while(a < -pi)
		a += 2*pi;
	//b = e_theta;
	while(b > pi)
		b -= 2*pi;
	while(b < -pi)
		b += 2*pi;
	return p, a, b;
}

int main(int argc, char **argv){
        ros::init(argc, argv, "pab_control");
        ros::NodeHandle nh;

        ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        ros::Subscriber sub = nh.subscribe("/robot_pose", 1000, &pose_callback);
	ros::Subscriber sub2 = nh.subscribe("/move_base_simple/goal", 1000, &goal_callback);

        ros::Rate rate(100);
        while(ros::ok()){
	        geometry_msgs::Twist vel;
		pab_calculation(n_x, n_y, n_theta, n_theta_pipi, g_x, g_y, g_theta);
		isGoal(n_x, n_y, n_theta_pipi, g_x, g_y, g_theta);
		if(isgoal==0){
			ROS_INFO("e_x= %f, e_y= %f, e_theta= %f", abs(g_x-n_x), abs(g_y-n_y), abs(e_theta));
			//ROS_INFO("not arrive");
			vel.linear.x = (kp*p)*adj;
			vel.angular.z = (ka*a+kb*b)*adj;
			//if(abs(e_x)<0.1 && abs(e_y)<0.1 && abs(b)>2){
			//	vel.linear.x = 0;
			//	vel.angular.z = (ka*a+kb*b)*adj;
			//}
			//pub.publish(vel);
			//ROS_INFO("v= %f, w= %f", vel.linear.x, vel.angular.z);
		}
		else if(isgoal==1){
			ROS_INFO("arrive");
			vel.linear.x = 0;
			vel.angular.z = 0;
			//pub.publish(vel);
		}
		pub.publish(vel);
	        ros::spinOnce();
	        rate.sleep();
        }
}
