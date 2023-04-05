#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/Bool.h"
#include <math.h>
#include <string.h>
#include <float.h>
#include <queue>
#include <stack>
#include <utility>
#include <set>
#include <vector>
#include <fstream>

using namespace std;

ifstream myfile;

#define pi acos(-1)
#define map_size 100
#define step 5
#define adj 8
#define INF 0x3f3f3f3f

const int dx[adj] = {step, step, 0, -step, -step, -step, 0, step};
const int dy[adj] = {0, step, step, step, 0, -step, -step, -step};


geometry_msgs::PoseStamped goal_pos;

typedef pair<int, int> point;
vector<point> goal = {{90, 80}, {30, 80}, {20, 10}, {90, 10}};
stack<point> path2;
queue<point> path;
bool publish = 0;
point ori;
int map_data[map_size][map_size];
int map_fn[map_size][map_size];
int map_gn[map_size][map_size];
/*	not dicovered = INF
	closed list = -1	*/
int map_parent[map_size][map_size];
/*	
	start = -1
	initial = 0;
	right = 1
	right-up = 2
	up = 3
	left-up = 4
	left = 5
	left-down = 6
	down = 7
	right-down = 8
*/
int goal_cnt;
bool calculated = false;


class mycomp {
	public:
		bool operator() (const point& lhs, const point& rhs) const {
			return map_fn[lhs.first][lhs.second] > map_fn[rhs.first][rhs.second];
		}
};

float dist(int i) {
	// calculate euclidean distance
	switch(i) {
		case 1:
		case 3:
		case 5:
		case 7:
			return step * 10;
		case 2:
		case 4:
		case 6:
		case 8:
			return step * 14;
		default: ROS_INFO_STREAM("WIERD :o!");
	}
	// return sqrt((p.first - y) * (p.first - y) + (p.second - x) * (p.second - x));
}

int cal_hn(point &cur) {
	// heuristic function for h(n)
	// calculate manhatton distance
	return (abs(cur.first - goal[goal_cnt].first) + abs(cur.second - goal[goal_cnt].second)) * 10;
}

void Astar() {
	// ROS_INFO_STREAM("ENTER A*");

	int tmp_x, tmp_y;
	int tmp_gn, tmp_fn;
	bool k = 0;
	point cur;
	priority_queue<point, vector<point>, mycomp> Q;

	Q.push(make_pair(ori.first, ori.second));
	map_fn[ori.first][ori.second] = cal_hn(ori);
	map_gn[ori.first][ori.second] = 0;
	map_parent[ori.first][ori.second] = -1;	// start node
	// ROS_INFO_STREAM("start: " << Q.top().first << ", " << Q.top().second);

	while (!Q.empty()) {

		cur = Q.top();
		if (cur == goal[goal_cnt]) {
			while (map_parent[cur.first][cur.second] != -1) {

				path2.push(cur);
				switch(map_parent[cur.first][cur.second]) {
					case 1:
						cur.second -= step;
						break;
					case 2:
						cur.first -= step;
						cur.second -= step;
						break;
					case 3:
						cur.first -= step;
						break;
					case 4:
						cur.first -= step;
						cur.second += step;
						break;
					case 5:
						cur.second += step;
						break;
					case 6:
						cur.first += step;
						cur.second += step;
						break;
					case 7:
						cur.first += step;
						break;
					case 8:
						cur.first += step;
						cur.second -= step;
						break;
				}
			}

			memset(map_fn, INF, map_size * map_size * sizeof(int));
			memset(map_gn, INF, map_size * map_size * sizeof(int));
			memset(map_parent, 0, map_size * map_size * sizeof(int));

			ori.first = goal[goal_cnt].first;
			ori.second = goal[goal_cnt++].second;
			break;
		}

		Q.pop();

		// iterate adjacent node
		for (int i = 0; i < adj; i++) {

			tmp_y = cur.first + dy[i];
			tmp_x = cur.second + dx[i];

			if (tmp_y > 99 || tmp_y < 0 || tmp_x > 99 || tmp_y < 0) continue;
			k = false;
			
			if (map_data[tmp_y][tmp_x] == 0) {
				if (map_gn[tmp_y][tmp_x] < 0) continue; // in closed list
				
				tmp_gn = map_gn[cur.first][cur.second] + dist(i+1);
				if (map_gn[tmp_y][tmp_x] == INF) {	// not in open list
					Q.push(make_pair(tmp_y, tmp_x));
					k = 1;
				}
				else {	// in open list
					if (tmp_gn < map_gn[tmp_y][tmp_x])	k = 1;	// in open list, smaller cost
				}

				if (k) {	// update fn
					map_parent[tmp_y][tmp_x] = i+1;
					map_gn[tmp_y][tmp_x] = tmp_gn;
					map_fn[tmp_y][tmp_x] = tmp_gn + cal_hn(cur);
				}
			}
		}
		map_gn[cur.first][cur.second] *= -1;
	}
}



void sub_callback(const nav_msgs::OccupancyGrid &msg) {
	
	// save map data
	int cnt = 0;
	int tmp;
	for (int i = map_size - 1; i >= 0; i--) {
		for (int j = 0; j < map_size; j++) {
			// map_data[i][j] = msg.data[cnt++];
			myfile >> tmp ;
			map_data[i][j] = tmp;
			// ROS_INFO_STREAM(map_data[i][j]);
		}
	}
	
	// save origin
	ori.first = 50; //msg.info.origin.position.y;
	ori.second = 50; //msg.info.origin.position.x;

	memset(map_fn, INF, map_size * map_size * sizeof(int));
    memset(map_gn, INF, map_size * map_size * sizeof(int));
    memset(map_parent, 0, map_size * map_size * sizeof(int));

	for (int i = 0; i < goal.size(); i++) {
		Astar();
	
		while (!path2.empty()) {
			ROS_INFO_STREAM(path2.top().first << ", " << path2.top().second);
			path.push(path2.top());
			path2.pop();
		}
	}
	ROS_INFO_STREAM("CALCUALTED");
	calculated = true;
	return;
}

void sub2_callback(const std_msgs::Bool &msg) {
	
	tf2::Quaternion myQuaternion;
	point cur;
	float angle;
	// if (publish) return;
	if (path.empty()) ROS_INFO_STREAM("------PATH IS EMPTY-------");
	if (!path.empty() && msg.data == 0 && calculated) {
		ROS_INFO_STREAM(path.front().first << ", " << path.front().second);
		cur = path.front();
		path.pop();
		angle = atan2(path.front().first - cur.first, path.front().second - cur.second);
		myQuaternion.setRPY(0, 0, angle);
		myQuaternion = myQuaternion.normalize();
		goal_pos.pose.position.y = (cur.first-50);
		goal_pos.pose.position.x = (cur.second-50);
		goal_pos.pose.position.y /= 10;
		goal_pos.pose.position.x /= 10;
		goal_pos.pose.orientation.x = myQuaternion.getX();
		goal_pos.pose.orientation.y = myQuaternion.getY();
		goal_pos.pose.orientation.z = myQuaternion.getZ();
		goal_pos.pose.orientation.w = myQuaternion.getW();
		// publish = 1;
	}
}



int main(int argc, char *argv[]) {

	myfile.open("/home/peaba/peaba_ws/src/lab3/src/processed_map.txt");
	if (!myfile.is_open()) ROS_INFO_STREAM("open file failed");
	else ROS_INFO_STREAM("open file successed");

	ros::init(argc, argv, "A_star");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pos", 1000);
	ros::Subscriber sub = nh.subscribe("/map", 1000, sub_callback);
	ros::Subscriber sub2 = nh.subscribe("/tracking", 1000, sub2_callback);

	ros::Rate rate(50);
	
	while (ros::ok()) {
		
		// pub.publish(res);
		// if (publish) {
			pub.publish(goal_pos);
		// 	publish = 0;
		// }
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
