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
typedef enum {
    PLANNER_STATE_LANE_FOLLOW,
    PLANNER_STATE_LANE_CHANGE_PREPARE,
    PLANNER_STATE_LANE_CHANGE_RIGHT,
    PLANNER_STATE_LANE_CHANGE_LEFT
} path_planner_state_t;

path_planner_state_t path_planner_state = PLANNER_STATE_LANE_FOLLOW;

vector<double> calculate_speed_and_acc(vector<vector<double>> pts, double dt) {
    auto x0 = pts[0][0];
    auto y0 = pts[0][1];
    auto x1 = pts[1][0];
    auto y1 = pts[1][1];
    auto x2 = pts[2][0];
    auto y2 = pts[2][1];

    auto v0 = sqrt(pow(x1-x0,2) + pow(y1-y0,2))/dt;
    auto v1 = sqrt(pow(x2-x1,2) + pow(y2-y1,2))/dt;

    auto a0 = (v1-v0)/dt;

    auto theta = atan2(y2-y1, x2-x1);

    return {v0, a0, theta};
}

/*
 * credit https://www.codeproject.com/Articles/17998/Some-simple-numerical-methods-in-C
 * Secant method for solving equation F(x) = 0
 * Input:
 * x0 - the first initial approximation of the solution
 * x1 - the second initial approximation of the solution
 * Output:
 * x - the resulted approximation of the solution
 * Return:
 * The number of iterations passed
 */
#define MAXITER 10000
#define error 0.00001
int SecantMethodForEquation(double& x, double x0, double x1, std::function<double(double)> F)
{
    int n = 2;

    while( ( fabs(F(x1)) > error ) && ( n <= MAXITER ) )
    {
        x = x1 - (F(x1) * (x1 - x0)) / (F(x1) - F(x0));
        x0 = x1;
        x1 = x;

        n++;
    }

    return n;
}

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
    auto next_x_vals = new vector<double>;
    auto next_y_vals = new vector<double>;
    auto retval = new vector<vector<double>>;

    if (path_planner_state != PLANNER_STATE_LANE_FOLLOW) {
        if (previous_path_x.size() > 10) {
            printf("keep executing lane change\n");
            // keep executing lane change
            for (int i = 0; i < previous_path_x.size(); ++i)
            {
                next_x_vals->push_back(previous_path_x[i]);
                next_y_vals->push_back(previous_path_y[i]);
            }

            retval->push_back(*next_x_vals);
            retval->push_back(*next_y_vals);

            return *retval;
        } else {
            cout << "points left " << previous_path_x.size() << endl;
            // return to lane following
            path_planner_state = PLANNER_STATE_LANE_FOLLOW;
        }
    }

    double car_x     = car_state[0];
    double car_y     = car_state[1];
    double car_s     = car_state[2];
    double car_d     = car_state[3];
    double car_yaw   = deg2rad(car_state[4]);
    double car_speed = car_state[5]/2.24;
    static int lane = 1;

    const float dt = 0.02; // s
    const float max_acc = 4.5; // m/s/s
    const float max_jerk = 40; // m/s/s/s
    const float max_speed = 47; // mph

    float target_speed = max_speed / 2.24; // m/s
    float target_jerk = 0;

    // Look at if there is a car in front
    float crt_lane_centre = 2+4*lane;
    for(int i=0; i<sensor_fusion.size(); i++){
        float d = sensor_fusion[i][6];
        if (d < (crt_lane_centre+2) && d > (crt_lane_centre-2)) {
            // car in my lane
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double v = sqrt(vx*vx + vy*vy);
            double s = sensor_fusion[i][5] + v * dt * previous_path_x.size();

            float current_dist =  sensor_fusion[i][5]- car_s;
            float future_dist = s - previous_path_end_state[0];

            if ((future_dist > 0 && future_dist < 30) ||
                (current_dist*future_dist<0)) {
                target_speed = v*0.9;
                printf("car in front set speed to %f\n",  target_speed);
            }
        }
    }

    // think about changing lanes
    int left_lane = lane - 1;
    int right_lane = lane + 1;
    bool left_lane_blocked = false;
    bool right_lane_blocked = false;

    // confine car in the 3 lanes on the right
    if (lane == 0) {
        left_lane_blocked = true;
    } else if (lane == 2) {
        right_lane_blocked = true;
    }

    vector<double> lane_speed = {1000, 1000, 1000};

    // only consider change lane if driving too slowly
    if (target_speed < 0.8*(max_speed/2.24)) {
        for(int i=0; i<sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6];
            int bogie_lane = d/4;

            // check for possible collision in the next 3 seconds
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double v = sqrt(vx*vx + vy*vy);
            double bogie_future_s = sensor_fusion[i][5] + v * 3;

            float current_dist = sensor_fusion[i][5]- car_s;
            float future_dist = bogie_future_s - car_s - car_speed * 3;

            // printf("bogie_lane %u car_speed %f current_dist %f future_dist %f\n", bogie_lane, car_speed, current_dist, future_dist);

            if ((abs(future_dist) < 15) || (abs(current_dist) < 15) ||
                (current_dist*future_dist<0)) {
                if (bogie_lane == left_lane && bogie_lane >= 0) {
                    // bogie in the lane to the left
                    // printf("left_lane_blocked\n");
                    left_lane_blocked = true;
                } else if (bogie_lane == right_lane && bogie_lane < 3) {
                    // bogie in the lane to the right
                    // printf("right_lane_blocked\n");
                    right_lane_blocked = true;
                }
            }

            // bogie in front
            if (current_dist > 0 && v < lane_speed[bogie_lane]) {
                lane_speed[bogie_lane] = v;
            }
        }
    }

    // printf("lane speed ");
    // for (int i = 2; i >= 0; --i)
    // {
    //     printf("%f ", lane_speed[i]);
    // }
    // printf(" %i %i %i %i %i", left_lane_blocked, right_lane_blocked, left_lane, lane, right_lane);
    // printf("\n");
    // decide to change lane
    if ((!left_lane_blocked) && (lane_speed[left_lane] > lane_speed[lane])) {
        printf("PLANNER_STATE_LANE_CHANGE_LEFT\n");
        path_planner_state = PLANNER_STATE_LANE_CHANGE_LEFT;
    } else if ((!right_lane_blocked) && (lane_speed[right_lane] > lane_speed[lane])) {
        printf("PLANNER_STATE_LANE_CHANGE_RIGHT\n");
        path_planner_state = PLANNER_STATE_LANE_CHANGE_RIGHT;
    }

    // generate path using spline
    tk::spline s;
    vector<double> ptsx, ptsy;
    int prev_path_size = previous_path_x.size();

    // get 3 points from preious path
    if (prev_path_size >= 3) {
        if (path_planner_state == PLANNER_STATE_LANE_FOLLOW) {
            // add 3 points from previous path
            for (int i = prev_path_size-3; i<prev_path_size; i++) {
                ptsx.push_back(previous_path_x[i]);
                ptsy.push_back(previous_path_y[i]);
            }
        } else { // executing lane change
            // add the first 3 points from previous path and trim the rest
            for (int i = 0; i<3; i++) {
                ptsx.push_back(previous_path_x[i]);
                ptsy.push_back(previous_path_y[i]);
            }
        }
    } else { // not enough data in previous path
        // add a previous way point as anchor
        vector<double> sd = getFrenet(car_x, car_y, car_yaw,
                                      map_waypoints_x,
                                      map_waypoints_y);

        vector<double> next_wp = getXY(sd[0]-30, 2+lane*4,
                                       map_waypoints_s,
                                       map_waypoints_x,
                                       map_waypoints_y);
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);

        // add car current position as anchor points
        ptsx.push_back(car_x);
        ptsy.push_back(car_y);
    }

    // generate the future points in a different lane
    if (path_planner_state == PLANNER_STATE_LANE_CHANGE_LEFT) {
        lane--;
    } else if (path_planner_state == PLANNER_STATE_LANE_CHANGE_RIGHT) {
        lane++;
    }

    // add 3 future points
    for (int i = 0; i < 3; i ++) {
        int k = ptsx.size()-1;
        float theta = atan2(ptsy[k]-ptsy[k-1], ptsx[k]-ptsx[k-1]);
        vector<double> sd = getFrenet(ptsx[k], ptsy[k], theta,
                                      map_waypoints_x,
                                      map_waypoints_y);
        vector<double> next_wp = getXY(sd[0]+30, 2+lane*4,
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
        ptsx[i] = dx * cos(-car_yaw) - dy * sin(-car_yaw);
        ptsy[i] = dx * sin(-car_yaw) + dy * cos(-car_yaw);
    }

    // calculate the spline
    s.set_points(ptsx, ptsy);

    // start constructing path
    int n_pts = 50;
    if (path_planner_state != PLANNER_STATE_LANE_FOLLOW) {
        n_pts = 200; // generate more points for lane change
    }

    double x_start, y_start;
    // static variable to save car state at end of path
    static float path_end_speed = 0;
    static float path_end_acc = 0;

    // add points from previous path
    if (prev_path_size >= 3) {
        int no_of_pts_to_add = 0;
        if (path_planner_state == PLANNER_STATE_LANE_FOLLOW) {
            // add all previous points to path
            no_of_pts_to_add = prev_path_size;
        } else { // changing lane
            // 3 previous points to path
            no_of_pts_to_add = 3;
        }

        // add points from previous path
        for(int i=0; i<no_of_pts_to_add; i++){
            next_x_vals->push_back(previous_path_x[i]);
            next_y_vals->push_back(previous_path_y[i]);
        }
        n_pts -= no_of_pts_to_add;

        // transform the end of previous path into car coordinate
        // and start new points generation from end of previous path
        auto dx = previous_path_x[no_of_pts_to_add-1] - car_x;
        auto dy = previous_path_y[no_of_pts_to_add-1] - car_y;
        x_start = dx * cos(-car_yaw) - dy * sin(-car_yaw);
        y_start = dx * sin(-car_yaw) + dy * cos(-car_yaw);

        // calculate the path end speed and acc
        if (path_planner_state != PLANNER_STATE_LANE_FOLLOW) {
            vector<vector<double>> pts;
            for (int i=no_of_pts_to_add-3; i<no_of_pts_to_add; i++) {
                pts.push_back({(*next_x_vals)[i], (*next_y_vals)[i]});
            }
            auto retval = calculate_speed_and_acc(pts, dt);
            path_end_speed = retval[0];
            path_end_acc = retval[1];
        }
    } else {
        // start path generation at the car
        x_start = 0;
        y_start = 0;
        path_end_speed = car_speed;
        // not enough info to determine path_end_acc just
        // use value remembered from last call
    }

    for(int i=0; i<n_pts; i++) {
        // set target jerk
        if (path_end_speed > target_speed && path_end_acc > -max_acc) {
            target_jerk = (path_end_acc+max_acc > max_jerk)? -max_jerk:-(path_end_acc + max_acc);
        } else if (path_end_speed < target_speed && path_end_acc < max_acc) {
            target_jerk = (max_acc-path_end_acc > max_jerk)? max_jerk:max_acc-path_end_acc;
        } else {
            target_jerk = 0;
        }

        // update speed and acc from jerk
        path_end_acc += target_jerk * dt;
        path_end_speed += path_end_acc * dt;

        // figure out the desired x step to create the right speed using a numeric method
        double dx0 = 0;
        double dx1 = path_end_speed*dt;
        double d_s = path_end_speed * dt + path_end_acc * dt * dt / 2.0;
        double d_x = dx1;
        SecantMethodForEquation(d_x, dx0, dx1,
            [x_start, s, d_s] (double dx) {
                return distance(x_start, s(x_start), x_start+dx, s(x_start+dx)) - d_s;
            });

        // printf("d_x %f\n", d_x);
        x_start += d_x;
        y_start = s(x_start);

        // transform to world coordinates
        auto trans_x = car_x+x_start*cos(car_yaw)-y_start*sin(car_yaw);
        auto trans_y = car_y+x_start*sin(car_yaw)+y_start*cos(car_yaw);

        // add points to path
        // printf("%f, %f, %f, %f\n", x_start, y_start, path_end_speed, path_end_acc);
        next_x_vals->push_back(trans_x);
        next_y_vals->push_back(trans_y);
    }

    retval->push_back(*next_x_vals);
    retval->push_back(*next_y_vals);

    return *retval;
}
