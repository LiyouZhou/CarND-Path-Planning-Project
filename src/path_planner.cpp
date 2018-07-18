#include <math.h>
#include <iostream>
#include "path_planner.h"
#include "spline.h"

using namespace std;

// For converting back and forth between radians and degrees.
extern double pi();
extern double deg2rad(double x);
extern double rad2deg(double x);
extern int NextWaypoint(double x, double y, double theta,
                        const vector<double> &maps_x,
                        const vector<double> &maps_y);
extern vector<double> getXY(double s, double d,
                            const vector<double> &maps_s,
                            const vector<double> &maps_x,
                            const vector<double> &maps_y);
extern double distance(double x1, double y1, double x2, double y2);
extern vector<double> getFrenet(double x, double y, double theta,
                                const vector<double> &maps_x,
                                const vector<double> &maps_y);

vector<vector<double>> plan_path(vector<double> car_state,
                                 vector<double> previous_path_x,
                                 vector<double> previous_path_y,
                                 vector<double> previous_path_end_state,
                                 vector<vector<double>> sensor_fusion,
                                 vector<double> map_waypoints_x,
                                 vector<double> map_waypoints_y,
                                 vector<double> map_waypoints_s,
                                 vector<double> map_waypoints_dx,
                                 vector<double> map_waypoints_dy)
{
    // cout << "planning path" << endl;

    auto next_x_vals = new vector<double>;
    auto next_y_vals = new vector<double>;
    auto retval = new vector<vector<double>>;

    double car_x     = car_state[0];
    double car_y     = car_state[1];
    double car_s     = car_state[2];
    double car_d     = car_state[3];
    double car_yaw   = deg2rad(car_state[4]);
    double car_speed = car_state[5];

    tk::spline s;
    vector<double> ptsx, ptsy;
    int prev_path_size = prev_path_size;

    if (prev_path_size >= 2) {
        // add two points from previous path
        for (int i = prev_path_size-2; i<prev_path_size; i++) {
            ptsx.push_back(previous_path_x[i]);
            ptsy.push_back(previous_path_y[i]);
        }
    } else {
        // add a previous way point
        vector<double> sd = getFrenet(car_x, car_y, car_yaw,
                                      map_waypoints_x,
                                      map_waypoints_y);

        vector<double> next_wp = getXY(sd[0]-30, sd[1],
                                       map_waypoints_s,
                                       map_waypoints_x,
                                       map_waypoints_y);
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);

        // add car current position as way points
        ptsx.push_back(car_x);
        ptsy.push_back(car_y);
    }

    // add 3 future points
    int lane = 1;
    for (int i = 0; i < 3; i ++) {
        int k = ptsx.size()-1;
        float theta = atan2(ptsy[k]-ptsy[k-1], ptsx[k]-ptsx[k-1]);
        vector<double> sd = getFrenet(ptsx[k], ptsy[k], theta,
                                      map_waypoints_x,
                                      map_waypoints_y);

        vector<double> next_wp = getXY(sd[0]+30, sd[1],
                                       map_waypoints_s,
                                       map_waypoints_x,
                                       map_waypoints_y);

        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }

    // transform anchor points into car coordinate
    for (int i = 0; i< ptsx.size(); i++) {
        auto dx = ptsx[i] - car_x;
        auto dy = ptsy[i] - car_y;
        auto trans_x = dx * cos(-car_yaw) - dy * sin(-car_yaw);
        auto trans_y = dx * sin(-car_yaw) + dy * cos(-car_yaw);

        ptsx[i] = trans_x;
        ptsy[i] = trans_y;
    }
    s.set_points(ptsx, ptsy);

    // figure out the desired x step to create the right speed
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = distance(0, 0, target_x, target_y);

    int n_pts = 50;
    double x_start, y_start;
    if (prev_path_size >= 2) {
        for(int i=0; i<prev_path_size; i++){
            next_x_vals->push_back(previous_path_x[i]);
            next_y_vals->push_back(previous_path_y[i]);
        }
        n_pts -= prev_path_size;
        x_start = previous_path_x[prev_path_size-1];
        y_start = previous_path_y[prev_path_size-1];

        // transform into car coordinate
        auto dx = previous_path_x[prev_path_size-1] - car_x;
        auto dy = previous_path_y[prev_path_size-1] - car_y;
        x_start = dx * cos(-car_yaw) - dy * sin(-car_yaw);
        y_start = dx * sin(-car_yaw) + dy * cos(-car_yaw);

    } else {
        x_start = 0;
        y_start = 0;
    }

    const float dt = 0.02; // s
    const float max_acc = 4; // m/s/s
    const float max_jerk = 40; // m/s/s/s
    const float max_speed = 48; // mph

    float target_speed = max_speed / 2.24; // m/s
    float target_jerk = 0;

    for(int i=0; i<sensor_fusion.size(); i++){
        float d = sensor_fusion[i][6];
        float crt_lane_centre = 2+4*lane;
        if (d < (crt_lane_centre+2) && d > (crt_lane_centre-2)) {
            // car in my lane
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double v = sqrt(vx*vx + vy*vy);
            double s = sensor_fusion[i][5] + v * dt * next_x_vals->size();

            float current_dist =  sensor_fusion[i][5]- car_s;
            float future_dist = s - previous_path_end_state[0];

            if ((future_dist > 0 && future_dist < 30) ||
                (current_dist*future_dist<0)) {
                target_speed = v*0.9;
            }
        }
    }

    static float path_end_speed = 0;
    static float path_end_acc = 0;

    for(int i=0; i<n_pts; i++){
        if (path_end_speed > target_speed) {
            target_jerk = -max_jerk;
        } else {
            target_jerk = max_jerk;
        }

        path_end_acc += target_jerk * dt;
        if (path_end_acc > max_acc) path_end_acc = max_acc;
        if (path_end_acc < -max_acc) path_end_acc = -max_acc;
        path_end_speed += path_end_acc * dt;

        float d_s = path_end_speed * dt + path_end_acc * dt * dt / 2.0;
        float dx = target_x * d_s / target_dist;

        double next_x = x_start + (i+1)*dx;
        double next_y = s(next_x);

        // transform to world coordinates
        auto trans_x = car_x+next_x*cos(car_yaw)-next_y*sin(car_yaw);
        auto trans_y = car_y+next_x*sin(car_yaw)+next_y*cos(car_yaw);

        // add points to path
        printf("%f, %f, %f, %f\n", trans_x, trans_y, path_end_speed, path_end_acc);
        next_x_vals->push_back(trans_x);
        next_y_vals->push_back(trans_y);
    }

    retval->push_back(*next_x_vals);
    retval->push_back(*next_y_vals);

    // cout << "planning done" << endl;

    return *retval;
}
