#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <stdbool.h>
#include <bits/stdc++.h>
using namespace std;

#define pi M_PI

#define SIZE 10000
#define X 100
#define Y 100
#define ROW 100
#define COL 100
#define error 0.05
int map_data[X][Y];
int temp[X][Y];
int map_data_test[10000];
double path_1[1000];

geometry_msgs::Twist car_position;
geometry_msgs::Twist goal_position;

float n_x, n_y, n_theta;
float origin_x, origin_y, resolution, width, height, path_x, path_y;
float g_x, g_y, g_x1, g_x2, g_y1, g_y2, g_theta, g_theta_pipi;
int path_num;
int path_length;
int which_goal;
int tempx;
int tempy;
int goal_num;
//std_msgs::Float64MultiArray path_copy;

typedef pair<int, int> Pair;
typedef pair<double, pair<int, int> > pPair;
struct cell {
	int parent_i, parent_j;
	double f, g, h;
};

Pair start = make_pair(50, 50);
Pair dest = make_pair(80, 90);//3 4

bool isValid(int row, int col)
{
	return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

bool isUnBlocked(int map_data[][COL], int row, int col)
{
	int inf=3;//inflation
	if (row==1 || row==98 || col==1 || col==98)
		return false;	
//	}
	//if (map_data[row][col] == 0)
	//if (map_data[row][col] == 0 && map_data[row-2][col] == 0 && map_data[row+2][col] == 0 && map_data[row][col-2] == 0 && map_data[row][col+2] == 0)
	if (map_data[row][col] == 0 && map_data[row-inf][col] == 0 && map_data[row+inf][col] == 0 && map_data[row][col-inf] == 0 && map_data[row][col+inf] == 0 && map_data[row-inf][col-inf] == 0 && map_data[row+inf][col-inf] == 0 && map_data[row-inf][col+inf] == 0 && map_data[row+inf][col+inf] == 0)
		return (true);
	else
		return (false);
}

bool isDestination(int row, int col, Pair dest)
{
	if (row == dest.first && col == dest.second)
		return (true);
	else
		return (false);
}

double calculateHValue(int row, int col, Pair dest)
{
	return ((double)sqrt((row - dest.first) * (row - dest.first) + (col - dest.second) * (col - dest.second)));
}


void tracePath(cell cellDetails[][COL], Pair dest)
{
	printf("\nThe Path is ");
	int row = dest.first;
	int col = dest.second;

	stack<Pair> Path;

	while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
		Path.push(make_pair(row, col));
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}
	Path.push(make_pair(row, col));
	path_num=0;
	while (!Path.empty()) {
		pair<int, int> p = Path.top();
		Path.pop();
		//ROS_INFO("-> (%d,%d) ", p.first, p.second);
		path_1[2*path_num]=p.first;
		path_1[2*path_num+1]=p.second;
		//ROS_INFO("pathx[%d]: %d, pathy[%d]: %d", path_num, path_1[2*path_num], path_num, path_1[2*path_num+1]);		
		//printf("x: %f ", path_1[2*path_num]);
		//printf("y: %f ", path_1[2*path_num+1]);		
		path_num++;
		if(p.first==dest.first && p.second==dest.second){
			path_length=2*path_num;
			printf("length= %d", path_length);
			break;
		}
		//printf("%d", path_num);	
	}
	return;
}

void pose_callback(const geometry_msgs::Twist &msg) {
	n_x = msg.linear.x;
	n_y = msg.linear.y;
	n_theta = msg.angular.z;
	while(n_theta > pi)
		n_theta -= 2*pi;
	while(n_theta < -pi)
		n_theta += 2*pi;
/*	if(abs(n_x) < error && abs(n_y) < error){
		which_goal = 1;
		//change_goal(which_goal);
		start = make_pair(50, 50);
		dest = make_pair(80, 90);
		printf("startx1= %d, starty1= %d", start.first, start.second);
	}
	else if(abs(n_x-3) < error && abs(n_y-4) < error){
		which_goal = 2;
		start = make_pair((int)((n_x+5)*10), (int)((n_y+5)*10));
		dest = make_pair(80, 30);
		printf("startx2= %d, starty2= %d", start.first, start.second);
	}
	else if(abs(n_x-3) < error && abs(n_y+2) < error){
		which_goal = 3;
		//change_goal(which_goal);
		start = make_pair((int)((n_x+5)*10), (int)((n_y+5)*10));
		dest = make_pair(10, 20);
		printf("startx3= %d, starty3= %d", start.first, start.second);
	}
	else if(abs(n_x+4) < error && abs(n_y+3) < error){
		which_goal = 4;
		//change_goal(which_goal);
		start = make_pair((int)((n_x+5)*10), (int)((n_y+5)*10));
		dest = make_pair(10, 90);
		printf("startx4= %d, starty4= %d", start.first, start.second);
	}
	else if(abs(n_x+4) < error && abs(n_y-4) < error){
		which_goal = 5;
		//change_goal(which_goal);
	}
	ROS_INFO("whichgoal= %d", which_goal);
	//change_goal(which_goal);
	//ROS_INFO("TEMPX: %d, TEMPY= %d", tempx, tempy);
*/
}

/*void goal_callback(const geometry_msgs::PoseStamped &msg) {
	g_x = msg.pose.position.x;
	g_y = msg.pose.position.y;
	g_theta = atan2(2*(msg.pose.orientation.w*msg.pose.orientation.z+msg.pose.orientation.x*msg.pose.orientation.y), 1-2*(msg.pose.orientation.z*msg.pose.orientation.z-msg.pose.orientation.y*msg.pose.orientation.y));
//	ROS_INFO("gx= %f, gy= %f, gth= %f", g_x, g_y, g_theta);
}*/

void map_callback(const nav_msgs::OccupancyGrid msg) {	 
    	origin_x = msg.info.origin.position.x;
  	origin_y = msg.info.origin.position.y;
   	resolution = msg.info.resolution;
    	width = msg.info.width;
    	height = msg.info.height;
	int size = 0;
	int i = 0;
	int j = 0;
	for(j = 0; j < Y ; j++ ){
		for(i = 0; i < X; i++){
			map_data[i][j] = msg.data[size];
			size++;
			//ROS_INFO("mapdata[%d][%d]: %d\n", i, j, map_data[i][j]);
		}		
	}
	//aStarSearch(map_data, start, dest);
	//for(int i=0; i<100; i++)
	//	ROS_INFO("map [%d][10]: %d", i,map_data[i][10]);
	//ROS_INFO("mappp: %d", map_data[0][0]);
	//ROS_INFO("mappp: %d", map_data[10][45]);
}

void aStarSearch(int map_data[][COL], Pair start, Pair dest)
{
	if (isValid(start.first, start.second) == false) {
		printf("Source is invalid\n");
		return;
	}

	if (isValid(dest.first, dest.second) == false) {
		printf("Destination is invalid\n");
		return;
	}

	if (isUnBlocked(map_data, start.first, start.second) == false || isUnBlocked(map_data, dest.first, dest.second) == false) {
		printf("Source or the destination is blocked\n");
		return;
	}

	if (isDestination(start.first, start.second, dest) == true) {
		printf("We are already at the destination\n");
		return;
	}

	bool closedList[ROW][COL];
	memset(closedList, false, sizeof(closedList));

	cell cellDetails[ROW][COL];

	int i, j;

	for (i = 0; i < ROW; i++) {
		for (j = 0; j < COL; j++) {
			cellDetails[i][j].f = FLT_MAX;
			cellDetails[i][j].g = FLT_MAX;
			cellDetails[i][j].h = FLT_MAX;
			cellDetails[i][j].parent_i = -1;
			cellDetails[i][j].parent_j = -1;
		}
	}

	i = start.first, j = start.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;

	set<pPair> openList;

	openList.insert(make_pair(0.0, make_pair(i, j)));

	bool foundDest = false;

	while (!openList.empty()) {
		pPair p = *openList.begin();

		openList.erase(openList.begin());

		i = p.second.first;
		j = p.second.second;
		closedList[i][j] = true;

		/*
		Generating all the 8 successor of this cell

			N.W   N   N.E
			  \   |   /
			   \  |  /
			W----Cell----E
			   /  |  \
			  /   |   \
			S.W   S   S.E

		Cell-->Popped Cell (i, j)
		N --> North	 (i, j+1)
		S --> South	 (i, j-1)
		E --> East	 (i+1, j)
		W --> West		 (i-1, j)
		N.E--> North-East (i+1, j+1)
		N.W--> North-West (i-1, j+1)
		S.E--> South-East (i+1, j-1)
		S.W--> South-West (i-1, j-1)*/

		// To store the 'g', 'h' and 'f' of the 8 successors
		double gNew, hNew, fNew;

		//----------- 1st Successor (North) ------------

		if (isValid(i, j + 1) == true) {
			if (isDestination(i, j + 1, dest) == true) {
				cellDetails[i][j+1].parent_i = i;
				cellDetails[i][j+1].parent_j = j;
				printf("The destination cell is found1\n");
				tracePath(cellDetails, dest);
				//tracePath(cellDetails, dest, sub_goal_point_x, sub_goal_point_y);
				foundDest = true;
				return;
			}
			else if (closedList[i][j+1] == false && isUnBlocked(map_data, i, j+1) == true) {
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j+1, dest);
				fNew = gNew + hNew;
				if (cellDetails[i][j+1].f == FLT_MAX || cellDetails[i][j+1].f > fNew) {
					openList.insert(make_pair(fNew, make_pair(i, j+1)));
					cellDetails[i][j+1].f = fNew;
					cellDetails[i][j+1].g = gNew;
					cellDetails[i][j+1].h = hNew;
					cellDetails[i][j+1].parent_i = i;
					cellDetails[i][j+1].parent_j = j;
				}
			}
		}

		//----------- 2nd Successor (South) ------------

		if (isValid(i, j-1) == true) {
			if (isDestination(i, j-1, dest) == true) {
				cellDetails[i][j-1].parent_i = i;
				cellDetails[i][j-1].parent_j = j;
				printf("The destination cell is found2\n");
				tracePath(cellDetails, dest);
				//tracePath(cellDetails, dest, sub_goal_point_x, sub_goal_point_y);
				foundDest = true;
				return;
			}
			else if (closedList[i][j-1] == false && isUnBlocked(map_data, i, j-1) == true) {
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j-1, dest);
				fNew = gNew + hNew;
				if (cellDetails[i][j-1].f == FLT_MAX || cellDetails[i][j-1].f > fNew) {
					openList.insert(make_pair(fNew, make_pair(i, j-1)));
					cellDetails[i][j-1].f = fNew;
					cellDetails[i][j-1].g = gNew;
					cellDetails[i][j-1].h = hNew;
					cellDetails[i][j-1].parent_i = i;
					cellDetails[i][j-1].parent_j = j;
				}
			}
		}

		//----------- 3rd Successor (East) ------------

		if (isValid(i+1, j) == true) {
			if (isDestination(i+1, j, dest) == true) {
				cellDetails[i+1][j].parent_i = i;
				cellDetails[i+1][j].parent_j = j;
				printf("The destination cell is found3\n");
				tracePath(cellDetails, dest);
				//tracePath(cellDetails, dest, sub_goal_point_x, sub_goal_point_y);
				foundDest = true;
				return;
			}
			else if (closedList[i+1][j] == false && isUnBlocked(map_data, i+1, j) == true) {
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i+1, j, dest);
				fNew = gNew + hNew;
				if (cellDetails[i+1][j].f == FLT_MAX || cellDetails[i+1][j].f > fNew) {
					openList.insert(make_pair(fNew, make_pair(i+1, j)));
					cellDetails[i+1][j].f = fNew;
					cellDetails[i+1][j].g = gNew;
					cellDetails[i+1][j].h = hNew;
					cellDetails[i+1][j].parent_i = i;
					cellDetails[i+1][j].parent_j = j;
				}
			}
		}

		//----------- 4th Successor (West) ------------

		if (isValid(i-1, j) == true) {
			if (isDestination(i-1, j, dest) == true) {
				cellDetails[i-1][j].parent_i = i;
				cellDetails[i-1][j].parent_j = j;
				printf("The destination cell is found4\n");
				tracePath(cellDetails, dest);
				//tracePath(cellDetails, dest, sub_goal_point_x, sub_goal_point_y);
				foundDest = true;
				return;
			}
			else if (closedList[i-1][j] == false && isUnBlocked(map_data, i-1, j) == true) {
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i-1, j, dest);
				fNew = gNew + hNew;
				if (cellDetails[i-1][j].f == FLT_MAX || cellDetails[i-1][j].f > fNew) {
					openList.insert(make_pair(fNew, make_pair(i-1, j)));
					cellDetails[i-1][j].f = fNew;
					cellDetails[i-1][j].g = gNew;
					cellDetails[i-1][j].h = hNew;
					cellDetails[i-1][j].parent_i = i;
					cellDetails[i-1][j].parent_j = j;
				}
			}
		}

		//----------- 5th Successor (North-East)

		if (isValid(i + 1, j + 1) == true) {
			if (isDestination(i + 1, j + 1, dest) == true) {
				cellDetails[i + 1][j + 1].parent_i = i;
				cellDetails[i + 1][j + 1].parent_j = j;
				printf("The destination cell is found5\n");
				tracePath(cellDetails, dest);
				//tracePath(cellDetails, dest, sub_goal_point_x, sub_goal_point_y);
				foundDest = true;
				return;
			}
			else if (closedList[i + 1][j + 1] == false && isUnBlocked(map_data, i + 1, j + 1) == true) {
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i + 1, j + 1, dest);
				fNew = gNew + hNew;
				if (cellDetails[i + 1][j + 1].f == FLT_MAX || cellDetails[i + 1][j + 1].f > fNew) {
					openList.insert(make_pair(fNew, make_pair(i + 1, j + 1)));
					cellDetails[i + 1][j + 1].f = fNew;
					cellDetails[i + 1][j + 1].g = gNew;
					cellDetails[i + 1][j + 1].h = hNew;
					cellDetails[i + 1][j + 1].parent_i = i;
					cellDetails[i + 1][j + 1].parent_j = j;
				}
			}
		}

		//----------- 6th Successor (North-West)

		if (isValid(i - 1, j + 1) == true) {
			if (isDestination(i - 1, j + 1, dest) == true) {
				cellDetails[i - 1][j + 1].parent_i = i;
				cellDetails[i - 1][j + 1].parent_j = j;
				printf("The destination cell is found6\n");
				tracePath(cellDetails, dest);
				//tracePath(cellDetails, dest, sub_goal_point_x, sub_goal_point_y);
				foundDest = true;
				return;
			}
			else if (closedList[i - 1][j + 1] == false && isUnBlocked(map_data, i - 1, j + 1) == true) {
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i - 1, j + 1, dest);
				fNew = gNew + hNew;
				if (cellDetails[i - 1][j + 1].f == FLT_MAX || cellDetails[i - 1][j + 1].f > fNew) {
					openList.insert(make_pair(fNew, make_pair(i - 1, j + 1)));
					cellDetails[i - 1][j + 1].f = fNew;
					cellDetails[i - 1][j + 1].g = gNew;
					cellDetails[i - 1][j + 1].h = hNew;
					cellDetails[i - 1][j + 1].parent_i = i;
					cellDetails[i - 1][j + 1].parent_j = j;
				}
			}
		}

		//----------- 7th Successor (South-East)

		if (isValid(i + 1, j - 1) == true) {
			if (isDestination(i + 1, j - 1, dest) == true) {
				cellDetails[i + 1][j - 1].parent_i = i;
				cellDetails[i + 1][j - 1].parent_j = j;
				printf("The destination cell is found7\n");
				tracePath(cellDetails, dest);
				//tracePath(cellDetails, dest, sub_goal_point_x, sub_goal_point_y);
				foundDest = true;
				return;
			}
			else if (closedList[i + 1][j - 1] == false && isUnBlocked(map_data, i + 1, j - 1) == true) {
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i + 1, j - 1, dest);
				fNew = gNew + hNew;
				if (cellDetails[i + 1][j - 1].f == FLT_MAX || cellDetails[i + 1][j - 1].f > fNew) {
					openList.insert(make_pair(fNew, make_pair(i + 1, j - 1)));
					cellDetails[i + 1][j - 1].f = fNew;
					cellDetails[i + 1][j - 1].g = gNew;
					cellDetails[i + 1][j - 1].h = hNew;
					cellDetails[i + 1][j - 1].parent_i = i;
					cellDetails[i + 1][j - 1].parent_j = j;
				}
			}
		}

		//----------- 8th Successor (South-West)

		if (isValid(i - 1, j - 1) == true) {
			if (isDestination(i - 1, j - 1, dest) == true) {
				cellDetails[i - 1][j - 1].parent_i = i;
				cellDetails[i - 1][j - 1].parent_j = j;
				printf("The destination cell is found8\n");
				tracePath(cellDetails, dest);
				//tracePath(cellDetails, dest, sub_goal_point_x, sub_goal_point_y);
				foundDest = true;
				return;
			}
			else if (closedList[i - 1][j - 1] == false && isUnBlocked(map_data, i - 1, j - 1) == true) {
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i - 1, j - 1, dest);
				fNew = gNew + hNew;
				if (cellDetails[i - 1][j - 1].f == FLT_MAX || cellDetails[i - 1][j - 1].f > fNew) {
					openList.insert(make_pair(fNew, make_pair(i - 1, j - 1)));
					cellDetails[i - 1][j - 1].f = fNew;
					cellDetails[i - 1][j - 1].g = gNew;
					cellDetails[i - 1][j - 1].h = hNew;
					cellDetails[i - 1][j - 1].parent_i = i;
					cellDetails[i - 1][j - 1].parent_j = j;
				}
			}
		}
	}
	if (foundDest == false)
		printf("Failed to find the Destination Cell\n");

	return;
}

int main(int argc, char **argv){
        ros::init(argc, argv, "A_star_peaba");
        ros::NodeHandle nh;

        //ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Publisher pub2 = nh.advertise<std_msgs::Float64MultiArray>("/sub_goal", 1000);
        ros::Subscriber sub = nh.subscribe("/robot_pose", 1000, &pose_callback);
	ros::Subscriber sub2 = nh.subscribe("/map", 1000, &map_callback);

        ros::Rate rate(100);
	
        while(ros::ok()){
		std_msgs::Float64MultiArray path_copy;

		printf("ros ok\n");

		start = make_pair(50, 50);
		dest = make_pair(80, 90);
		aStarSearch(map_data, start, dest);
		for(int i=0; i<path_length; i++){
			//printf("%f\n ", path_1[i]);
			path_copy.data.push_back(path_1[i]);
		}

		start = make_pair(80, 90);
		dest = make_pair(80, 30);
		aStarSearch(map_data, start, dest);
		for(int i=0; i<path_length; i++){
			//printf("%f\n ", path_1[i]);
			path_copy.data.push_back(path_1[i]);
		}


		start = make_pair(80, 30);
		dest = make_pair(10, 20);
		aStarSearch(map_data, start, dest);
		for(int i=0; i<path_length; i++){
			//printf("%f\n ", path_1[i]);
			path_copy.data.push_back(path_1[i]);
		}

		start = make_pair(10, 20);
		dest = make_pair(10, 90);
		aStarSearch(map_data, start, dest);
		for(int i=0; i<path_length; i++){
			//printf("%f\n ", path_1[i]);
			path_copy.data.push_back(path_1[i]);
		}


/*		aStarSearch(map_data, start, dest);
		for(int i=0; i<path_length; i++){
			//printf("%f\n ", path_1[i]);
			path_copy.data.push_back(path_1[i]);
			printf("%f\n ", path_copy.data[i]);
		}
*/

		pub2.publish(path_copy);
	        ros::spinOnce();
	        rate.sleep();
        }
}






