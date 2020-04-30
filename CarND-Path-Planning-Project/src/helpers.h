#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"

// for convenience
using std::string;
using std::vector;

///GLOBALVARIABLES: 
int lane = 1;         // Lane 0 => left, Lane 1 => mid, Lane 2 => right
double ref_vel = 0.0; // Desired vel of the car
string state = "KL";  // Initial condition of the car (Keep Lane)

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

/**
 * PREVIOUS: Prediction function   
 **/ 
/*bool get_preditiction_cars(vector<vector<double>> sensor_fusion, int &lane, int prev_size, double car_s){
  bool too_close = false;
  //find ref_v to use
  for(int i = 0; i < sensor_fusion.size(); i++){
    //car is in my lane 
    float d = sensor_fusion[i][6];
    if(d < (2+4*lane+2) && d > (2+4*lane-2)){
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];

      //Where the other car will be in the future
      check_car_s+=((double) prev_size*.02*check_speed); 

      //Check s calues greater than mine and s gap
      if((check_car_s > car_s) && (check_car_s-car_s) < 30){
        too_close = true;
        lane=2;
      }
    }
  }
  return too_close;
}*/

/**
 * TODO: Implement the Prediction function that will predict the movements of the other cars  
 **/ 
vector <bool> get_preditiction_cars(vector<vector<double>> sensor_fusion, int lane, int prev_size, double car_s){
  bool too_close = false;
  bool front_left_car = false;
  bool front_mid_car = false;
  bool front_right_car = false;
  
  //find ref_v to use
  for(int i = 0; i < sensor_fusion.size(); i++){
    float d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx+vy*vy);
    double check_car_s = sensor_fusion[i][5];

    //Where the other car will be in the future
    check_car_s+=((double) prev_size*.02*check_speed);
    
    // Left lane in the road
    if(d < 4 && d > 0){
      if((check_car_s > car_s) && (check_car_s-car_s) < 30){
        front_left_car = true;
        if(lane == 0){
          too_close = true;
        }
      }
    }
    // Middle lane
    else if(d < 8 && d > 4){
      if((check_car_s > car_s) && (check_car_s-car_s) < 30){
        front_mid_car = true;
        if(lane == 1){
          too_close = true;
        }
      }
    }
    //Right Lane
    else if(d < 12 && d > 8){
      if((check_car_s > car_s) && (check_car_s-car_s) < 30){
        front_right_car = true;
        if(lane == 2){
          too_close = true;
        }
      }
    }
  }
  return {front_left_car,front_mid_car,front_right_car,too_close};
}

/**
 * TODO: Implement the Trajectory function that will build a path for the autonomous car  
 **/ 
vector<vector<double>> get_trajectory(vector<double> &map_waypoints_x,vector<double> &map_waypoints_y, vector<double> &map_waypoints_s, int prev_size,
                                      double car_x, double car_y, double car_s, double car_yaw,vector <double> previous_path_x,vector <double> previous_path_y,int lane,string possible_state){

  //Create a listt of widely spaced (x,y) waypoints, evely spaced at 30m
  // Later we will interpolate theses way points with a spline and fill it in with more points 
  vector<double> ptsx;
  vector<double> ptsy;

  //References x,y,yaw states
  //either we will reference the starting point as where the car is or at the previous paths and point
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);


  // if previous size is almost empty 
  if(prev_size < 2){
    //Use two points that make the path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  // Use the previous path's end point as starting reference
  else{
    
    //Redefine reference state as previous path end point 
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

    //Use two points that make path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

  }

  if(possible_state == "KL"){
    lane = lane;
  }
  else if(possible_state == "LCL"){
    lane--;
  }
  else if(possible_state == "LCR"){
    lane++;
  }
  
  //In Frenet add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);          

  //Convert to cars frame reference
  for(int i = 0; i < ptsx.size(); i++){
    //shift car reference angle to 0 degrees
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
  }

  //create a spline 
  tk::spline s;

  //set (x,y) points to the spline
  s.set_points(ptsx,ptsy);

  //Define the actual (x,y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  //Start with all of the previous path points from last time
  for(int i = 0; i < previous_path_x.size(); i++){
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  //Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

  double x_add_on = 0;

  // Calculating the remaining points to reach 50
  // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  for(int i = 1; i <= 50 - previous_path_x.size(); i++){ 
    double N = (target_dist/(0.02*ref_vel/2.24));
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Convert it into world frame
    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  return {next_x_vals,next_y_vals};
}

/**
 * TODO: Implement the successor_states function that store all the possible next states of a FSM.  
 **/
vector<string> successor_states(string current_state) {
  // Store all the sucessor states of the current state
  vector<string> states;
  
  // Sucessors of Keep Lane state
  if(current_state == "KL") {
    states.push_back("LCL");
    states.push_back("LCR");
    states.push_back("KL");
  } 
   
  // Sucessors for Lane Change Left state
  else if(current_state == "LCL") {
    states.push_back("LCL");
    states.push_back("KL");
  }
 
  // Sucessors for Lane Change Right state
  else if(current_state == "LCR") {
    states.push_back("LCR");
    states.push_back("KL");
  }
  return states;
}

#endif  // HELPERS_H