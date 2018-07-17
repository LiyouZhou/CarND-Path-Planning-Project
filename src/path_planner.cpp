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
    cout << "planning path" << endl;
    auto next_x_vals = new vector<double>;
    auto next_y_vals = new vector<double>;
    auto retval = new vector<vector<double>>;

    double car_x     = car_state[0];
    double car_y     = car_state[1];
    double car_s     = car_state[2];
    double car_d     = car_state[3];
    double car_yaw   = deg2rad(car_state[4]);
    double car_speed = car_state[5];

    // double pos_x;
    // double pos_y;
    // double angle;

    // for(int i = 0; i < path_size; i++)
    // {
    //     next_x_vals->push_back(previous_path_x[i]);
    //     next_y_vals->push_back(previous_path_y[i]);
    // }

    // if(path_size == 0)
    // {
    //     pos_x = car_x;
    //     pos_y = car_y;
    //     angle = deg2rad(car_yaw);
    // }
    // else
    // {
    //     pos_x = previous_path_x[path_size-1];
    //     pos_y = previous_path_y[path_size-1];

    //     double pos_x2 = previous_path_x[path_size-2];
    //     double pos_y2 = previous_path_y[path_size-2];
    //     angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    // }

    // double dist_inc = 0.5;
    // for(int i = 0; i < 50-path_size; i++)
    // {
    //     next_x_vals->push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
    //     next_y_vals->push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
    //     pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
    //     pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
    // }

    tk::spline s;

    // set spline anchor points
    vector<double> ptsx, ptsy;

    int prev_path_size = previous_path_x.size();

    // add two points from previous path
    if (prev_path_size >= 2) {
        for (int i = prev_path_size-2; i<prev_path_size; i++) {
            // cout << "x " << previous_path_x[i] << " y " << previous_path_y[i] << endl;

            // add to list
            ptsx.push_back(previous_path_x[i]);
            ptsy.push_back(previous_path_y[i]);
        }
    } else {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsy.push_back(prev_car_y);

        ptsx.push_back(car_x);
        ptsy.push_back(car_y);
    }

    // add 3 future points
    double x, y, theta;
    if (prev_path_size >= 2) {
        x = previous_path_x[prev_path_size-1];
        y = previous_path_y[prev_path_size-1];
        double dx = previous_path_x[prev_path_size-1] - previous_path_x[prev_path_size-2];
        double dy = previous_path_y[prev_path_size-1] - previous_path_y[prev_path_size-2];
        theta = atan2(y, x);
    } else {
        x = car_x;
        y = car_y;
        theta = car_yaw;
    }

    int lane = 1;
    int prev_r = 0;
    for (int i = 0; i < 3; i ++) {
        // get the next way point
        int r = prev_r;
        while (r == prev_r) {
            x += cos(theta);
            y += sin(theta);
            r = NextWaypoint(x, y, theta,
                             map_waypoints_x,
                             map_waypoints_y);
        }
        prev_r = r;

        double x_prev = x;
        double y_prev = y;

        x = map_waypoints_x[r];
        y = map_waypoints_y[r];
        theta = atan2(y-y_prev, x-x_prev);

        // lock car in lane
        vector<double> xy = getFrenet(x, y, theta,
                                      map_waypoints_x,
                                      map_waypoints_y);
        xy = getXY(xy[0], 2+lane*4,
                   map_waypoints_s,
                   map_waypoints_x,
                   map_waypoints_y);
        x = xy[0];
        y = xy[1];

        // cout << "x " << x << " y " << y << "theta" << theta << endl;
        ptsx.push_back(x);
        ptsy.push_back(y);
    }

    for (int i = 0; i< ptsx.size(); i++) {
        cout << "spline " << ptsx[i] << " " << ptsy[i];
        // transform into car coordinate
        auto dx = ptsx[i] - car_x;
        auto dy = ptsy[i] - car_y;
        auto trans_x = dx * cos(-car_yaw) - dy * sin(-car_yaw);
        auto trans_y = dx * sin(-car_yaw) + dy * cos(-car_yaw);

        ptsx[i] = trans_x;
        ptsy[i] = trans_y;

        cout << " -> " << ptsx[i] << " " << ptsy[i] << endl;
    }
    s.set_points(ptsx, ptsy);

    // figure out the desired x step to create the right speed
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = distance(0, 0, target_x, target_y);
    const double ref_vel = 49.5 / 2.24;
    float dx = target_x * 0.02 * ref_vel / target_dist;

    int n_pts = 50;
    double x_start, y_start;
    if (prev_path_size >= 2) {
        for(int i=0; i<previous_path_x.size(); i++){
            next_x_vals->push_back(previous_path_x[i]);
            next_y_vals->push_back(previous_path_y[i]);
        }
        n_pts -= previous_path_x.size();
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

    printf("car pos %f %f %f\n", car_x, car_y, car_yaw);
    for(int i=0; i<n_pts; i++){
        // cout << i << endl;

        double next_x = x_start + (i+1)*dx;
        double next_y = s(next_x);
        auto trans_x = car_x + next_x * cos(car_yaw) - next_y * sin(car_yaw);
        auto trans_y = car_y + next_x * sin(car_yaw) + next_y * cos(car_yaw);

        // vector<double> xy = getXY(next_s,
        //                        next_d,
        //                        map_waypoints_s,
        //                        map_waypoints_x,
        //                        map_waypoints_y);
        // cout << "push" << endl;

        vector<double> xy = getFrenet(trans_x, trans_y, car_yaw, map_waypoints_x, map_waypoints_y);
        cout << "transx " << trans_x << " transy " << trans_y << " " << xy[0] << " " << xy[1] << endl;

        next_x_vals->push_back(trans_x);
        next_y_vals->push_back(trans_y);
    }


    retval->push_back(*next_x_vals);
    retval->push_back(*next_y_vals);

    cout << "planning done" << endl;

    return *retval;
}