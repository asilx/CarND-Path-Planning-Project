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

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  int lane = 1;
  int desired_lane = lane;
  double ref_vel = 0;
  bool prepare_lane_change = false;
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
  
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane, &desired_lane, &prepare_lane_change]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();
          
          if(prev_size > 2)
          {
            car_s = end_path_s;
          }
          
          float proximity[3];
          proximity[0] = 999;
          proximity[1] = 999;
          proximity[2] = 999;
          
          bool proximity_alert[3];
          
          proximity_alert[0] = false;
          proximity_alert[1] = false;
          proximity_alert[2] = false;
          
          bool infront[3];
          infront[0] = false;
          infront[1] = false;
          infront[2] = false;
          
          float lane_speed[3];
          
          lane_speed[0] = -1;
          lane_speed[1] = -1;
          lane_speed[2] = -1;
          
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            //if(d < 4*lane + 4 && d > 4 * lane)
            //{
            float sx = sensor_fusion[i][5];
              
            float vx = sensor_fusion[i][3];
            float vy = sensor_fusion[i][4];
              
            float v = sqrt(vx * vx + vy * vy);
              
              
            sx += ((double) prev_size * 0.02 * v);
            
            if(proximity[(int)d/4] > abs(sx - car_s))
            {
              lane_speed[(int)d/4] = v;
              proximity[(int)d/4] = abs(sx - car_s);
              
              if(proximity[(int)d/4] < 22 && (int)d/4 != lane)
              {
                proximity_alert[(int)d/4] = true;
              
                if(sx > car_s) 
                   infront[(int)d/4] = true; 
              }
              else if(proximity[(int)d/4] < 30 && (int)d/4 == lane)
              {
                proximity_alert[(int)d/4] = true;
              
                if(sx > car_s) 
                  infront[(int)d/4] = true; 
              }
            }
            
          }
          
          if (prepare_lane_change)
          {
            if (ref_vel <= lane_speed[desired_lane])
            {
              prepare_lane_change = false;
            }
            ref_vel -= .224;
          }
          else if (proximity_alert[lane] && infront[lane])
          {
            if (lane + 1 < 3 && !proximity_alert[lane + 1])
            {
              if(proximity[lane + 1] < 50 && abs(proximity[lane + 1] - proximity[lane]) > 10 && infront[lane + 1] && ref_vel > lane_speed[lane + 1 ])
              {
                prepare_lane_change = true;
                desired_lane = lane + 1;
              }
              else if (abs(proximity[lane + 1] - proximity[lane]) > 10  || !infront[lane + 1] && ref_vel > lane_speed[lane + 1] )
              {
                prepare_lane_change = false;
                desired_lane = lane + 1;
              }
              ref_vel -= .224;
              
            }
            else if(lane -1 >= 0 && !proximity_alert[lane - 1])
            {
              if(proximity[lane - 1] < 50 && abs(proximity[lane - 1] - proximity[lane]) > 10 && infront[lane - 1] && ref_vel > lane_speed[lane - 1])
              {
                prepare_lane_change = true;
                desired_lane = lane-1;
              }
              else if (abs(proximity[lane - 1] - proximity[lane]) > 10 || !infront[lane - 1] && ref_vel > lane_speed[lane - 1] )
              {
                prepare_lane_change = false;
                desired_lane = lane-1;
              }
              ref_vel -= .224;
            }
            else if (proximity[lane] < 10) //emergency break
              ref_vel = lane_speed[lane];
            else
              ref_vel -= .224;
          }
          else if (ref_vel < 49.5)
            ref_vel += .224;
          
          vector<double> spline_pts_x;
          vector<double> spline_pts_y;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          vector<double> trajpoint1 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> trajpoint2 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> trajpoint3 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          double prevx = car_x - cos(car_yaw);
          double prevy = car_y - sin(car_yaw);
          
          if(prev_size >= 2)
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            prevx = previous_path_x[prev_size - 2];
            prevy = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - prevy, ref_x - prevx);
          }
          
          spline_pts_x.push_back(prevx);
          spline_pts_y.push_back(prevy);
          spline_pts_x.push_back(ref_x);
          spline_pts_y.push_back(ref_y);
          spline_pts_x.push_back(trajpoint1[0]);
          spline_pts_y.push_back(trajpoint1[1]);
          spline_pts_x.push_back(trajpoint2[0]);
          spline_pts_y.push_back(trajpoint2[1]);
          spline_pts_x.push_back(trajpoint3[0]);
          spline_pts_y.push_back(trajpoint3[1]); 
          
          
          for(int i = 0; i < spline_pts_x.size(); i++)
          {
            double transform_x = spline_pts_x[i] - ref_x;
            double transform_y = spline_pts_y[i] - ref_y;
            
            spline_pts_x[i] = (transform_x * cos(-1 * ref_yaw)) - (transform_y * sin(-1 * ref_yaw));
            spline_pts_y[i] = (transform_x * sin(-1 * ref_yaw)) + (transform_y * cos(-1 * ref_yaw));
          }
          
          tk::spline s;
          s.set_points(spline_pts_x, spline_pts_y);
          
          for(int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
          
          double N = (target_dist/(0.02*ref_vel/2.24));
          double milestone_dist = target_x / N;
          
          for(int i = 0; i <= 50 - previous_path_x.size(); i++)
          {
            double x_point = (i+1) * milestone_dist;
            double y_point = s(x_point);
           
            
            double x_temp = x_point;
            double y_temp = y_point;
            
            x_point = (x_temp * cos(ref_yaw) - y_temp*sin(ref_yaw));
            y_point = (x_temp * sin(ref_yaw) + y_temp*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          if(!prepare_lane_change)
            lane = desired_lane;
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
