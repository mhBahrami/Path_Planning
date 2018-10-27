#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;
// Time elapsed into the future [sec]
const double TIME_ELAPSED = 0.02;
// "s" gap
const double S_GAP = 30.0;
// Maximum speed
const double MAX_SPEED = 49.5;
double MAX_ACC = 0.0;
double stayInLaneCheckPoint = 0.0;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mphToMps(double v) { return v / 2.237; }
double mpsToMph(double v) { return v * 2.237; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_first_of('}');
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
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp = static_cast<int>(maps_x.size() - 1);
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = static_cast<int>((prev_wp + 1) % maps_x.size());

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

double getD(const int laneIndex) {
  return 2 + 4 * laneIndex;
}

int getLane(const double d) {
  if (0 <= d && d < 4)
  {
    return 0;
  }
  else if (4 <= d && d < 8)
  {
    return 1;
  }
  else if (8 <= d && d < 12)
  {
    return 2;
  }

  return -1;
}

vector<bool> analyzeSensorFusionData(const vector<vector<double>> &sensor_fusion, const int lane, const double car_s,
                                     const int prev_size) {

  bool isThereAnyCarOnTheLeftAndAhead = false;
  bool isThereAnyCarAhead = false;
  bool isThereAnyCarOnTheRightAndAhead = false;
  for (auto &sf_data : sensor_fusion) {
    // sf_data format: [ id, x, y, vx, vy, s, d]
    double d = sf_data[6];
    int car_lane = getLane(d);
    if (car_lane < 0 || car_lane > 2) {
      continue;
    }
    // Find car speed.
    double vx = sf_data[3];
    double vy = sf_data[4];
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = sf_data[5];
    // If we're using previous path points, we're not quite there yet!
    // So, we can project s value outwards in time!
    check_car_s += (double) prev_size * TIME_ELAPSED * check_speed;

    if (car_lane == lane) {
      // There is a vehicle in the same lane as our car's lane
      // Check s values greater than mine and s gap to adjust car's speed
      isThereAnyCarAhead |= check_car_s  > car_s  && check_car_s - car_s < S_GAP * 3.0/4.0;
    } else {
      bool isThereAnyCarOnTheSideAndAhead =
          -1.0 / 2.0 * S_GAP < check_car_s - car_s
          && check_car_s - car_s < S_GAP * 1.1;
      if (car_lane + 1 == lane) {
        isThereAnyCarOnTheLeftAndAhead |= isThereAnyCarOnTheSideAndAhead;
      } else if (car_lane - 1 == lane) {
        isThereAnyCarOnTheRightAndAhead |= isThereAnyCarOnTheSideAndAhead;
      }
    }
  }

  return {
    isThereAnyCarOnTheLeftAndAhead,
    isThereAnyCarAhead,
    isThereAnyCarOnTheRightAndAhead
  };
}

void getBehaviour(const vector<bool> &isThereAnyCarCloseToMine, int &lane, double &ref_vel) {

  const bool isThereAnyCarOnTheLeftAndAhead = isThereAnyCarCloseToMine[0];
  const bool isThereAnyCarAhead = isThereAnyCarCloseToMine[1];
  const bool isThereAnyCarOnTheRightAndAhead = isThereAnyCarCloseToMine[2];

  if (stayInLaneCheckPoint > 50 * TIME_ELAPSED) {
    // Determine the next state behaviour based on the current state
    if (isThereAnyCarAhead) {
      // It means we will stock behind a car potentially!
      // Change the line if it's possible otherwise decrease the speed
      switch (lane) {
        case 0:
          if(!isThereAnyCarOnTheRightAndAhead) {
            // Go to the right lane!
            lane++;
            // Reset the check point
            stayInLaneCheckPoint = 0.0;
          }
          // Adjust the speed!
          ref_vel -= MAX_ACC;
          break;
        case 1:
          if(!isThereAnyCarOnTheLeftAndAhead)
          {
            // Regardless of what's on the right lane change to the left lane
            // Go to the left lane!
            lane--;
            // Reset the check point
            stayInLaneCheckPoint = 0.0;
          } else if (!isThereAnyCarOnTheRightAndAhead) {
            // Go to the right lane!
            lane++;
            // Reset the check point
            stayInLaneCheckPoint = 0.0;
          }
          // Adjust the speed!
          ref_vel -= MAX_ACC;
          break;
        case 2:
          if(!isThereAnyCarOnTheLeftAndAhead) {
            // Go to the left lane!
            lane--;
            // Reset the check point
            stayInLaneCheckPoint = 0.0;
          }
          // Adjust the speed!
          ref_vel -= MAX_ACC;
          break;
        default:
          // Adjust the speed!
          ref_vel -= MAX_ACC;
        }

    } else {
      // It means There is no car ahead and close to mine.
      // So, Keep driving and only watch out for the speed!
      // Adjust the speed
      if (ref_vel < MAX_SPEED) {
        ref_vel = min(MAX_SPEED, ref_vel + MAX_ACC);
        //speed_diff -= MAX_ACC;
      }

      // Sometimes the vehicle stocks in the lane #0 or #2
      // To avoid it, always try to back to center line if possible!
      if (lane != 1) { // if we are not on the center lane.
        if ((lane == 0 && !isThereAnyCarOnTheRightAndAhead /*|| isThereAnyCarOnTheRightBehind*/)
            || (lane == 2 && !isThereAnyCarOnTheLeftAndAhead /*|| isThereAnyCarOnTheLeftBehind*/)) {
          // Drive back to center lane #1
          lane = 1;
        }
      }
    }
  }
  ref_vel = min(MAX_SPEED, ref_vel);
  ref_vel = max(MAX_ACC, ref_vel);
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
  double max_s = 6945.554;

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

  // Set max acceleration
  MAX_ACC = mpsToMph(0.3 /*[m/sec]*/);
  // This is the car's lane to start driving
  int lane = 1;

  // Have a reference velocity to target
  double ref_vel = 0.0; //mph
  stayInLaneCheckPoint = 10.0; // To move the car faster at the beginning!

  h.onMessage([&ref_vel, &lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (!s.empty()) {
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
          // The data format for each car is: [ id, x, y, vx, vy, s, d]
          auto sensor_fusion = j[1]["sensor_fusion"];


          /// --- Begin ---
          int prev_size = previous_path_x.size();

          // It helps to prevent collision
          if(prev_size > 0) {
            car_s = end_path_s;
          }

          /// 1. Prediction
          vector<bool> isThereAnyCarCloseToMine2 = analyzeSensorFusionData(sensor_fusion, lane, car_s, prev_size);
          stayInLaneCheckPoint += TIME_ELAPSED;
          getBehaviour(isThereAnyCarCloseToMine2, lane, ref_vel);

          /// 2. Generate the safe trajectory to drive
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline and fill it in with more points that control sp
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference x, y, yaw states
          // Either we will reference the starting points as where the car is or at the previous paths end points
          double ref_x = 0.0;
          double ref_y = 0.0;
          double ref_yaw = 0.0;
          double ref_x_prev = 0.0;
          double ref_y_prev = 0.0;

          // If the previous state is almost empty, use the car point as starting reference
          if (prev_size < 2) {
            // Use two points that make the path tangent to the car
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
            ref_x_prev = ref_x - cos(ref_yaw);
            ref_y_prev = ref_y - sin(ref_yaw);
          }
          // Use the previous path's end points as starting reference
          else {
            // Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            ref_x_prev = previous_path_x[prev_size-2];
            ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }

          // Use two points that make the path together to the previous path's end points
          ptsx.push_back(ref_x_prev);
          ptsx.push_back(ref_x);

          ptsy.push_back(ref_y_prev);
          ptsy.push_back(ref_y);

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+1*S_GAP, getD(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+2*S_GAP, getD(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+3*S_GAP, getD(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Shift car reference angle to 0 degrees, it makes the math easier
          for (int k = 0; k < ptsx.size(); ++k) {
            // It is a shift and rotation
            double shift_x = ptsx[k] - ref_x;
            double shift_y = ptsy[k] - ref_y;

            ptsx[k] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[k] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // Create a spline object
          tk::spline spline_obj;
          // Set (x, y) points to the spline
          spline_obj.set_points(ptsx, ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_values;
          vector<double> next_y_values;

          // Start with all of the previous path points from last time
          for (int l = 0; l < prev_size; ++l) {
            next_x_values.push_back(previous_path_x[l]);
            next_y_values.push_back(previous_path_y[l]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = S_GAP;
          double target_y = spline_obj(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          // N * time_elapsed [sec] (time to get to the next point in the path) * v [m/sec] (car velocity) = target_dist [m]
          double N = target_dist / (TIME_ELAPSED * mphToMps(ref_vel));
          double delta_x = target_x / N;

          double x_add_on = 0.0;
          for (int m = 1; m <= 50 - prev_size; ++m) {
            double x_point = x_add_on + delta_x;
            double y_point = spline_obj(x_point);

            x_add_on = x_point;
            double x_store = x_point;
            double y_store = y_point;

            // Shift and rotate back to normal after rotating it earlier
            x_point = (x_store * cos(ref_yaw) - y_store * sin(ref_yaw));
            y_point = (x_store * sin(ref_yaw) + y_store * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_values.push_back(x_point);
            next_y_values.push_back(y_point);
          }

          /// --- End ---

          json msgJson;
          msgJson["next_x"] = next_x_values;
          msgJson["next_y"] = next_y_values;

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
