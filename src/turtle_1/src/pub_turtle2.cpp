#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <iomanip>
#include <math.h>

double vx=0, vy=0, omg=0;
bool ang = 0;
double pi = M_PI;


void sub_callback(const turtlesim::Pose &msg) {
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "position = (" << msg.x << ", " << msg.y << ", " << msg.theta << " ) " );
	double dx=msg.x-5.54;
	double dy=msg.y-5.54;
	double th=pi/2;
//	double x=msg.x;
//	double y=msg.y;
	double theta=msg.theta;
	if(dx+dy < 1 && theta==0 && omg==0){
//		ROS_INFO_STREAM("OMG: " << omg);
		ROS_INFO_STREAM("dx+dy1= " << dx+dy);
		vx=1;
		omg=0;
	}
	else if((dx+dy >= 1) && theta < th && theta>=0){
		ROS_INFO_STREAM("dx+dy2= " << dx+dy);
		omg=1;
		vx=0;
	}
	else if(theta >= th && dx+dy >= 1 && dx+dy < 2){
		vx=1;
		omg=0;
		ROS_INFO_STREAM("dx+dy3= " << dx+dy);
	}
	else if(dx+dy >= 2 && theta >= th && theta < 2*th){
		omg=1;
		ROS_INFO_STREAM("dx+dy4= " << dx+dy);
		ROS_INFO_STREAM("theta1: " << theta);
		vx=0;
	}
	else if(theta >= -2*th && dx+dy >= 2){
		vx=1;
		ROS_INFO_STREAM("theta2: " << theta);
		omg=0;
	}
	else if(theta <= -1*th && theta > -2*th && dx+dy <= 1){
		omg=1;
		ROS_INFO_STREAM("dx+dy5= " << dx+dy);	
		vx=0;
	}
	else if(theta <= 0 && theta > -1*th && dx+dy < 1 && dx+dy >= 0){
		vx=1;
		ROS_INFO_STREAM("theta3: " << theta);
		omg=0;
	}
	else if(dx+dy <= 0 && theta < 0 && theta > -1*th){
		omg=1;
		ROS_INFO_STREAM("theta4: " << theta);
		vx=0;
	}  
	else if(theta >= 0 && dx+dy < 0){
		omg=0;
		vx=1;
	}
}


int main(int argc, char **argv){
        ros::init(argc, argv, "run_rec2");
        ros::NodeHandle nh;
	
	//int secs =ros::Time::now().toSec();

        ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);
        ros::Subscriber sub = nh.subscribe("turtle1/pose", 0, &sub_callback);

        ros::Rate rate(1000);
        while(ros::ok()){
	        geometry_msgs::Twist msg;
	        msg.linear.x = vx;
		//msg.linear.y = vy;
		msg.angular.z = omg;
	        pub.publish(msg);
	        ros::spinOnce();
	        rate.sleep();
        }
}

