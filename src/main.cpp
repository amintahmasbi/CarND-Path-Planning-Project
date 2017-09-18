#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/Dense"

#include "spline.h"
#include "json.hpp"
#include "classifier.h"
#include "vehicle.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
        {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y); // DotProduct(x,n)/|n|^2
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

        }

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
        {
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

        }

// Check if a lane is free
bool isFree(Vehicle self, double buffer, map<int,Vehicle> &adjacent_lane)
{
  const double timeheadway = 0.1;
  bool free = true;
  for(map<int, Vehicle>::iterator ll_it = adjacent_lane.begin(); ll_it != adjacent_lane.end(); ++ll_it)
  {
    Vehicle side_car = ll_it->second;

    if ( (side_car.s + timeheadway*side_car.s_dot <= self.s - self.L - buffer ))// && (side_car.end_path_s >= self.end_path_s - self.L - buffer) )
    {
      free = false;
      break;
    }
    else if((side_car.s - self.L - buffer >= self.s + timeheadway*self.s_dot ))// && (side_car.end_path_s - self.L - buffer <= self.end_path_s ) )
    {
      free = false;
      break;
    }


  }

  return free;
}

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
  /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficient in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */

  double a_0 = start[0];
  double a_1 = start[1];
  double a_2 = 0.5 * start[2];

  double T2 = T*T;
  double T3 = T2*T;

  MatrixXd mat3(3,3);
  mat3 << T3, T2*T2, T2*T3,
      3*T2, 4*T3, 5*T2*T2,
      6*T, 12*T2, 20*T3;

  VectorXd v(3);
  v << end[0] - (start[0] + start[1]*T + 0.5 * start[2] * T2),
      end[1] - (start[1] + start[2] * T),
      end[2] - start[2];

  VectorXd a_3to5(3);
  a_3to5 = mat3.inverse() * v;

  return {a_0,a_1,a_2,a_3to5(0),a_3to5(1),a_3to5(2)};

}

// Load training states for NB classifier
vector<vector<double> > Load_State(string file_name)
{
    ifstream in_state_(file_name.c_str(), ifstream::in);

    vector< vector<double >> state_out;
    string line;

    while (getline(in_state_, line))
    {

      istringstream iss(line);
      vector<double> x_coord;
      string state1;
      string state2;
      string state3;
      string state4;
      getline(iss, state1, ',');
      x_coord.push_back(stod(state1));

      getline(iss, state2, ',');
      x_coord.push_back(stod(state2));

      getline(iss, state3, ',');
      x_coord.push_back(stod(state3));

      getline(iss, state4, ',');
      x_coord.push_back(stod(state4));

      state_out.push_back(x_coord);
    }
    return state_out;
}

// Load training labels for NB classifier
vector<string> Load_Label(string file_name)
{
    ifstream in_label_(file_name.c_str(), ifstream::in);
    vector< string > label_out;
    string line;
    while (getline(in_label_, line))
    {
      istringstream iss(line);
      string label;
      iss >> label;

      label_out.push_back(label);
    }
    return label_out;

}

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
  const double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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

  vector< vector<double> > X_train = Load_State("../pred_data/train_states.txt"); // {s,d,s_dot,d_dot}
  vector< string > Y_train  = Load_Label("../pred_data/train_labels.txt");

//  cout << "X_train number of elements " << X_train.size() << endl;
//  cout << "X_train element size " << X_train[0].size() << endl;
//  cout << "Y_train number of elements " << Y_train.size() << endl;

  GNB gnb = GNB();

  gnb.train(X_train, Y_train); // use:    string predicted = gnb.predict(coords); : left, right, keep

  // Start in lane 1;
  int lane = 1;
  const int lane_width = 4;
  // Have a reference velociy to targe
  double ref_vel = 0.0; // m/s
  const double speed_limit = 22.2; // m/s
  const double dt = 0.02;// timestep

  const double safety_buffer = 1.0;
  //radius of situational awareness
  const double awareness_horizon = 40.0;
  const double lookahead_horizon = 50.0; // Affect the sharpness of spline

  h.onMessage([&safety_buffer,&lane,&ref_vel,&awareness_horizon,&lookahead_horizon,&lane_width,
               &speed_limit,&dt,&gnb,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          vector<map<int, Vehicle>> remote_vehicles(3); // Should renew every cycle
          remote_vehicles.resize(3);



          //Store neighboring cars in awareness horizon and classify them based on their lane
          for (int i = 0; i < sensor_fusion.size(); ++i)
          {
            int remote_id = sensor_fusion[i][0];
            double remote_x = sensor_fusion[i][1];
            double remote_y = sensor_fusion[i][2];
            double remote_vx = sensor_fusion[i][3];
            double remote_vy = sensor_fusion[i][4];
            double remote_s = sensor_fusion[i][5];
            double remote_d = sensor_fusion[i][6];

            if (abs(car_s - remote_s) <= awareness_horizon )
            {

              int remote_lane = floor(remote_d / lane_width);

              if(remote_lane < 0 || remote_lane > 2)
              {
                continue;
              }
              //Predict state of remote vehicle
              double remote_speed = sqrt(remote_vx*remote_vx+remote_vy*remote_vy);
              double remote_next_x = remote_x + dt*remote_vx;
              double remote_next_y = remote_y + dt*remote_vy;


              double remote_next_theta = atan2(remote_next_y-remote_y, remote_next_x-remote_x);
              vector<double> nextFrenet = getFrenet(remote_next_x, remote_next_y, remote_next_theta, map_waypoints_x, map_waypoints_y);
              double remote_s_dot = (nextFrenet[0]-remote_s)/dt;
              double remote_d_dot = (nextFrenet[1]-remote_d)/dt;



              vector<double> data_point = {remote_s,remote_d,0,0};
              string predicted_state = gnb.predict(data_point); // TODO: better prediction model is needed!: s_dot, d_dot

              if(predicted_state.compare("keep") != 0)
              {
                cout << predicted_state << endl;
              }
              double remote_target_s = remote_s + lookahead_horizon*dt*remote_s_dot;
              double remote_target_d = lane_width/2 + lane_width*remote_lane; // TODO: should depend on prediction
              Vehicle vehicle(remote_lane, remote_s, remote_d, remote_speed,
                              remote_target_s, remote_target_d, predicted_state);
              vehicle.s_dot = remote_s_dot;
              vehicle.d_dot = remote_d_dot;
              vehicle.end_path_s = remote_s + prev_size*dt*remote_s_dot;

              remote_vehicles[remote_lane].insert(std::pair<int,Vehicle>(remote_id,vehicle));

            }
          }


          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Add the remaining points of the previous path to the current one: continues transition
          for(int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline and fill it in with more points
          // that control speed
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference x,y, yaw states
          // Either we will reference the stating point as where the car is or at the previous paths end point

          double ref_x;
          double ref_y;
          double ref_yaw;
          double prev_ref_x;
          double prev_ref_y;

          if(prev_size < 2)
          {
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);

            prev_ref_x = ref_x -  cos(ref_yaw);
            prev_ref_y = ref_y -  sin(ref_yaw);

          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            prev_ref_x = previous_path_x[prev_size-2];
            prev_ref_y = previous_path_y[prev_size-2];

            ref_yaw  = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

          }

          vector<double> prevFrenet = getFrenet(prev_ref_x,prev_ref_y,ref_yaw,map_waypoints_x,map_waypoints_y);
          double prev_s = prevFrenet[0];
          double prev_d = prevFrenet[1];

          double car_s_dot = (car_s - prev_s)/dt;
          double car_d_dot = (car_d - prev_d)/dt;

          ptsx.push_back(prev_ref_x);
          ptsx.push_back(ref_x);

          ptsy.push_back(prev_ref_y);
          ptsy.push_back(ref_y);

          //update state
          bool too_close = false;

          double target_speed = speed_limit;
          lane = floor(car_d / lane_width);
          if(lane < 0 || lane > 2)
          {
            lane = 1;
          }

          Vehicle ego(lane, car_s, car_d, car_speed, car_s + lookahead_horizon*dt*car_s_dot, lane_width/2+lane_width*lane, "keep");
          ego.s_dot = car_s_dot;
          ego.d_dot = car_d_dot;
          ego.end_path_s = end_path_s;
          int tmp_lane = lane;

          for(map<int, Vehicle>::iterator it = remote_vehicles[lane].begin(); it != remote_vehicles[lane].end(); ++it)
          {

            Vehicle veh = it->second;
            if ((veh.d < (lane_width/2 + lane_width*lane + lane_width/2)) &&
                veh.d > (lane_width/2 + lane_width*lane - lane_width/2)) // check if in lane
            {

              // check s values greater than mine and s gap
              if ((veh.s  >= ego.s - ego.L - safety_buffer))
              {
                //find empty adjacent lanes
                switch (lane) {
                  case 0:
                    if( (remote_vehicles[1].empty()) || isFree(ego, safety_buffer, remote_vehicles[1]) )
                    {
                      tmp_lane = 1;
                    }
                    else
                    {
                      too_close = true;
                      target_speed = veh.v;
                    }
                    break;
                  case 1:
                    if ( (remote_vehicles[0].empty()) || isFree(ego, safety_buffer, remote_vehicles[0]) )
                    {
                      tmp_lane = 0;
                    }
                    else if( (remote_vehicles[2].empty()) || isFree(ego, safety_buffer, remote_vehicles[2]) )
                    {
                      tmp_lane = 2;
                    }
                    else
                    {
                      too_close = true;
                      target_speed = veh.v;
                    }
                    break;
                  case 2:
                    if( (remote_vehicles[1].empty()) || isFree(ego, safety_buffer, remote_vehicles[1]) )
                    {
                      tmp_lane = 1;
                    }
                    else
                    {
                      too_close = true;
                      target_speed = veh.v;

                    }
                    break;
                  default:
                    cout << "out of lane!" << endl;
                    break;
                }

              }
            }
          }

          lane = tmp_lane;

          if (too_close)
          {
            if(ref_vel > target_speed)
            {
              ref_vel -= 0.1;
            }
          }
          else if (ref_vel < speed_limit)
          {
            ref_vel += 0.1;
          }


          vector<double> next_wp0 = getXY(car_s+lookahead_horizon, lane_width/2+(lane_width*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+2*lookahead_horizon, lane_width/2+(lane_width*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+3*lookahead_horizon, lane_width/2+(lane_width*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); ++i) {

            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x *cos(0-ref_yaw) - shift_y *sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw) + shift_y *cos(0-ref_yaw));

          }

          // create a spline
          tk::spline s;


          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          //Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = lookahead_horizon;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;


          //Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points

          for(int i = 1; i < lookahead_horizon - previous_path_x.size(); i++)
          {

            double N = (target_dist/(dt*(ref_vel+0.01)));// avoid divid by zero
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_tmp = x_point;
            double y_tmp = y_point;

            //rotate back to normal after rotating it earlier
            x_point = (x_tmp *cos(ref_yaw) - y_tmp *sin(ref_yaw));
            y_point = (x_tmp *sin(ref_yaw) + y_tmp *cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
      size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
















































































