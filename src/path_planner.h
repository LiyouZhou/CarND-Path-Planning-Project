#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

using namespace std;

vector<vector<double>> plan_path(vector<double> car_state,
	 							 vector<double> previous_path_x,
	 							 vector<double> previous_path_y,
	 							 vector<double> previous_path_end_state,
	 							 vector<vector<double>>  sensor_fusion,
	 							 vector<double> map_waypoints_x,
	 							 vector<double> map_waypoints_y,
								 vector<double> map_waypoints_s,
								 vector<double> map_waypoints_dx,
								 vector<double> map_waypoints_dy);

#endif
