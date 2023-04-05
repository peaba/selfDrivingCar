#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <stdbool.h>
#include <std_msgs/Int8.h>

#define kp 3
#define ka 8 
#define kb -3
#define adj 0.05

#define pi M_PI

#define error 0.05
float n_x, n_y, n_theta, n_theta_pipi;
float g_x, g_y, g_x1, g_x2, g_y1, g_y2, g_theta, g_theta_pipi;
float e_x, e_y, e_theta;
float p, a, b;
int isgoal;
int goal_num;
int which_goal;
std_msgs::Int8 msg;
int tempx;
int tempy;

void pose_callback(const geometry_msgs::Twist &msg) {
	n_x = msg.linear.x;
	n_y = msg.linear.y;
	n_theta = msg.angular.z;
	while(n_theta > pi)
		n_theta -= 2*pi;
	while(n_theta < -pi)
		n_theta += 2*pi;
	ROS_INFO("nx= %f, ny= %f, nth= %f", n_x, n_y, n_theta);
}

void sub_goal_callback(const std_msgs::Float64MultiArray &msg) {
	g_x1 = (msg.data[goal_num]/10)-4.9;
	g_y1 = (msg.data[goal_num+1]/10)-4.9;
	g_x2 = (msg.data[goal_num+2]/10)-4.9;
	g_y2 = (msg.data[goal_num+3]/10)-4.9;
	g_x = g_x1;
	g_y = g_y1;
	g_theta = atan2((g_y2-g_y1),(g_x2-g_x1));
	while(g_theta > pi)
		g_theta -= 2*pi;
	while(g_theta < -pi)
		g_theta += 2*pi;
	ROS_INFO("gx= %f, gy= %f, gth= %f", g_x, g_y, g_theta);

/*	if(abs(n_x-3) < error && abs(n_y-4) < error){
		//g_x = 3;
		//g_y = 4;
		goal_num = 0;
	}
	else if(abs(n_x-3) < error && abs(n_y+2) < error){
		//g_x = 3;
		//g_y = -2;
		goal_num = 0;
	}
	else if(abs(n_x+4) < error && abs(n_y+3) < error){
		//g_x = -4;
		//g_y = -3;
		goal_num = 0;
	}
	else if(abs(n_x+4) < error && abs(n_y-4) < error){
		//g_x = -4;
		//g_y = 4;
		goal_num = 0;
	}
*/
	/*else{
		g_x = g_x1;
		g_y = g_y1;
		goal_num = goal_num;
	}*/

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
        ros::init(argc, argv, "astar_control");
        ros::NodeHandle nh;

        ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        ros::Subscriber sub = nh.subscribe("/robot_pose", 1000, &pose_callback);
	ros::Subscriber sub2 = nh.subscribe("/sub_goal", 1000, &sub_goal_callback);

        ros::Rate rate(100);
        while(ros::ok()){
	        geometry_msgs::Twist vel;
		pab_calculation(n_x, n_y, n_theta, n_theta_pipi, g_x, g_y, g_theta);
		isGoal(n_x, n_y, n_theta_pipi, g_x, g_y, g_theta);
		if(isgoal==0){
			//ROS_INFO("e_x= %f, e_y= %f, e_theta= %f", abs(g_x-n_x), abs(g_y-n_y), abs(e_theta));
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
			//ROS_INFO("arrive");
			vel.linear.x = 0;
			vel.angular.z = 0;
			goal_num += 2;
			//pub.publish(vel);
		}
		pub.publish(vel);
	        ros::spinOnce();
	        rate.sleep();
        }
}
