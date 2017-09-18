#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

  struct collider{

    bool collision ; // is there a collision?
    double  time; // time collision happens

  };

  int L = 4; //The "collider" on the car measures 4.47x2.43

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  double s;

  double s_dot;

  double end_path_s;

  double v;

  double d;

  double d_dot;

  double target_d;

  double target_s;

  string state;

  /**
  * Constructor
  */
  Vehicle(int lane, double s, double d, double v, double target_s, double target_d, string state);

  /**
  * Destructor
  */
  virtual ~Vehicle();

//  void update_state(map<int, vector <vector<int> > > predictions);
//
//  void configure(vector<int> road_data);
//
//  string display();
//
//  void increment(int dt);
//
//  vector<int> state_at(int t);
//
//  bool collides_with(Vehicle other, int at_time);
//
//  collider will_collide_with(Vehicle other, int timesteps);
//
//  void realize_state(map<int, vector < vector<int> > > predictions);
//
//  void realize_constant_speed();
//
//  int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);
//
//  void realize_keep_lane(map<int, vector< vector<int> > > predictions);
//
//  void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);
//
//  void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);
//
//  vector<vector<int> > generate_predictions(int horizon);

};

#endif
