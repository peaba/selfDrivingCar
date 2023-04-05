#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <bits/stdc++.h>
#include<fstream>
#include <string>
#include<iostream>
using namespace std;

float v = 0.4; // must be greater than 0.25
std_msgs::Float64MultiArray waypoint;
#include "math.h"

ros::Publisher path_pub;

int i = 0;
void read_path_file(){

  float x, y;
  ifstream myFile;
  myFile.open("/home/peaba/peaba_ws/src/final_sim/src/path_final.txt");
  for( int g = 0; g < 42; g ++){
    myFile>>x>>y;
    // printf("%f %f\n", x, y);
    waypoint.data.push_back(x);
    waypoint.data.push_back(y);
    waypoint.data.push_back(0);
    waypoint.data.push_back(0.4);
    //if( y > 13.5 && y < 16) 
      //waypoint.data.push_back(v - 0.2);
    //else
      //waypoint.data.push_back(v);
  }
  float prev_angle = 0;
  int ss = waypoint.data.size();
  for( int j = 0; j < ss-4; j+=4){
    waypoint.data[j+2] = atan2( (waypoint.data[j+1+4] -  waypoint.data[j+1]),  (waypoint.data[j+4]-waypoint.data[j]));// + prev_angle)/2;
    //prev_angle = waypoint.data[j+2];
	if (waypoint.data[j+2] > 3.14/2 || waypoint.data[j+2] < -3.14/2) waypoint.data[j+3] = 0.3;
	else waypoint.data[j+3] = 0.4;
  }
waypoint.data[ss-2] = 0;
waypoint.data[ss-4] = 15.525;
waypoint.data[ss-3] = 14.765;


return;
}
int main(int argc, char** argv)
{
    
  ros::init(argc, argv, "path_sim");
  ros::NodeHandle nh;

  path_pub = nh.advertise<std_msgs::Float64MultiArray>("way_point", 1);

  ros::Rate loop_rate(30);
 read_path_file();
  int limit = 0;

  while (ros::ok() ){
    if( limit < 5)
     path_pub.publish(waypoint);
    loop_rate.sleep();
    limit ++;
  }
  ros::spin();
}
