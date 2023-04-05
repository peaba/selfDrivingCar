#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <iomanip>

#define pi 3.14159
#define cnt 80
const int v_x[8] = {1, 0, 0, 0, -1, 0, 0, 0};
const int v_y[8] = {0, 0, 1, 0, 0, 0, -1, 0};
int count  = 0;
int i = 0;
bool ang = 0;
void sub_callback(const turtlesim::Pose &msg) {
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "position = (" << msg.x << ", " << msg.y << ", " << msg.theta << " ) " );
	count++;
	if (ang == 0) {    
		ROS_INFO_STREAM("ANG = 0, " << ", i: " << i);
		switch (i) {
			case 0: {
//					if (msg.x > 7) i++, ang = 1;
					if (count % cnt == 0) {
						i++, ang = 1, count = 0;
						ROS_INFO_STREAM("ang: " << ang << ", i: " << i);
					}

				}break;
			case 2: {
//					if (msg.y > 7) i++, ang = 1;
					if (count % cnt == 0) i++, ang = 1, count = 0;

				}break;
			case 4: {
//					if (msg.x < 5) i++, ang = 1;
					if (count % cnt == 0) i++, ang = 1, count = 0;

				}break;
			case 6: {
//					if (msg.y < 5) i++, ang = 1;
					if (count % cnt == 0) i = 0, ang = 1, count = 0;

				}break;
			default: break;
//			case 8: i = 0; break;
		}

//		if (msg.x > 7 || msg.y < 3 || msg.x < 5 || msg.y > 5){
//		      ang = 1;
//	      	      i++;
//		}
	}		
	else {
		ROS_INFO_STREAM("ANG = 1" << ", i: " << i);
		i++;
		ang = 0;/*
		switch (i) {
			case 1: {
//					if (msg.theta > pi/2 - 0.1 && msg.theta < pi/2) {
					if (msg.theta > 1.5 && msg.theta < 1.6){
						i++, ang = 0;
					       	ROS_INFO_STREAM("TURN 1");
					}
				}break;
			case 3: {
					if (msg.theta > 3 && msg.theta < 3.1){
                                                i++, ang = 0;
                                                ROS_INFO_STREAM("TURN 2");
                                        }
					//if (msg.theta > pi - 0.1 && msg.theta < pi) i++, ang = 0;//, ROS_INFO_STREAM("TURN 2");
				}break;
			case 7: {
//					if (msg.theta > 0 - 0.1 && msg.theta < 0) i++, ang = 0;//, ROS_INFO_STREAM("TURN 3");
					if (msg.theta > -0.1 && msg.theta < 0){
                                                i++, ang = 0;
                                                ROS_INFO_STREAM("TURN 3");
                                        }
				}break;
			case 5: {
//					if (msg.theta > -1 * pi/2 - 0.1 && msg.theta < -1 * pi/2) i++, ang = 0;//, ROS_INFO_STREAM("TURN 4");
					if (msg.theta > -1.6 && msg.theta < -1.5){
                                                i++, ang = 0;
                                                ROS_INFO_STREAM("TURN 4");
                                        }
				}break;
			default: break;
		}*/
//		if (msg.theta == pi/2 || msg.theta == pi || msg.theta == -1*pi/2 || msg.theta == -1*pi) {
//			ang = 0;
//			i++;
//			if (i == 8) i = 0;
//		}
	}

}
int main(int argc, char **argv){
        ros::init(argc, argv, "run_rec");
        ros::NodeHandle nh;

        ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
        ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &sub_callback);

        ros::Rate rate(1000);
        while(ros::ok()){
                geometry_msgs::Twist msg;
                msg.linear.x = v_x[i];
		msg.linear.y = v_y[i];
		msg.angular.z = ang;
                pub.publish(msg);
                ros::spinOnce();
                rate.sleep();
        }
}

