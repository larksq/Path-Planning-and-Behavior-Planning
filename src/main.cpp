#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const double MAX_VEL = 49.5;
const double MAX_ACC = .224;
const double MAX_DEC = .448;
const int DISTANCE_AHEAD = 30;
const int LANE_WIDTH = 4;
const float D_COLLISION_BOUND = 1.5;
const float S_COLLISION_BOUND = 5; // should give enough distance to decrease
const float SIDE_COLLISION_BOUND = 25; // should give enough distance to decrease


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  double ref_vel = 0;


  // start in lane 1
//  int lane = 1;

  // reference velocity
//  double ref_vel = 0.0; //mph

  bool car_ahead = false;
  bool car_left = false;
  bool car_right = false;

  h.onMessage([&car_ahead, &car_left, &car_right, &ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                                                          uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int prev_size = previous_path_x.size();

          double car_s_t = car_s; // save the current position of the car

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
          bool left_clear = true;
          bool right_clear = true;

//          double car_s_tn = car_s + ref_vel * 0.02 * (double) prev_size / 50;

          for (auto & i : sensor_fusion)
          {
            float other_car_d = i[6];
            float d_bias = other_car_d - (2 + lane * 4);
            // current lane
            // Determine the speed of the other car
            double vx = i[3];
            double vy = i[4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double other_car_s = i[5];
            double other_car_s_tn = other_car_s; // s of other cars at t_n
            // Estimate the other car's position after executing previous trajectory
            other_car_s_tn += (double) prev_size * 0.02 * check_speed / 50;

//            if (abs(other_car_s_tn - car_s_t) < SIDE_COLLISION_BOUND){
            if (abs(other_car_s_tn - car_s) < SIDE_COLLISION_BOUND){
//              std::cout << d_bias << std::endl;

              if (d_bias>0) {
                if (d_bias < (D_COLLISION_BOUND + LANE_WIDTH))
                {
                  right_clear = false;
                  std::cout << "right traffic" << std::endl;
                }
              }
              else{
                if (d_bias * -1 < (D_COLLISION_BOUND + LANE_WIDTH)){
                  left_clear = false;
                  std::cout << "left traffic" << std::endl;
                }
              }
            }

            if (abs(d_bias) < D_COLLISION_BOUND) {
              if ((other_car_s_tn > car_s_t) && ((other_car_s - car_s) < S_COLLISION_BOUND)) {
                too_close = true;
              }
            }
            else{
              continue;
            }
          }

          if (too_close){
            std::cout << "too close" << std::endl;
            ref_vel -= MAX_ACC;
            if (left_clear && lane>0){
              lane -= 1;
            }
            else if (right_clear && lane<2)
            {
              lane += 1;
            }
            else{
              std::cout << "both sides traffic, hold" << std::endl;
            }
          }
          else if (ref_vel < MAX_VEL)
          {
            ref_vel += MAX_ACC;
          }


          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If previous size is almost empty, use car as starting reference
          if (prev_size < 2) {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
            // Use previous path's points as starting reference
          else {
            // Redefine reference state as previous path endpoint
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            // Use two points that make the path tangent to the previous path's endpoint
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+DISTANCE_AHEAD, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+(DISTANCE_AHEAD*2), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+(DISTANCE_AHEAD*3), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Now ptsx and ptsy has 6 pts: t-2, t-1, t-1 on 30m ahead, t-1 on 60m ahead, t-1 on 90m ahead

          for (int i=0; i<ptsx.size(); i++) {
            // transfer to t-1 local coordinates
            // Shift car angle reference to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          // Create a spline
          tk::spline s;

          // Set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for (int i=0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that the desired reference velocity is kept
          // we are setting a target at 30 at x, and cut the line to have several points
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // Fill up the rest of the path planner after filling it with previous points
          // Always 50 points will be output
          for (int i=0; i <= 50-previous_path_x.size(); i++) {

            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //Rotate back to normal after rotating earlier
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}