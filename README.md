# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving `+-10 MPH` of the `50 MPH` speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the `50 MPH` speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the `6946m` highway. Since the car is trying to go `50 MPH`, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over `10 m/s^2` and jerk that is greater than `10 m/s^3`.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  `[x,y,s,dx,dy]` values. `x` and `y` are the waypoint's map coordinate position, the `s` value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the Frenet `s` value, distance along the road, goes from `0` to `6945.554`.

### Details

1. The car uses a perfect controller and will visit every `(x,y)` point it receives in the list every `0.02` seconds. The units for the `(x,y)` points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The `(x,y)` point paths that the planner receives should not have a total acceleration that goes over `10 m/s^2`, also the jerk should not go over `50 m/s^3`. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a `0.02` second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.
2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. `previous_path_x`, and `previous_path_y` can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)

## Project Instructions and Rubric

### Compiling and executing the project

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Spline.h

A new file was added [src/spline.h](https://github.com/darienmt/CarND-Path-Planning-Project-P1/blob/master/scr/spline.h). It is the [Cubic Spline interpolation implementation](http://kluge.in-chemnitz.de/opensource/spline/): a single header file you can use splines instead of polynomials. It was a great suggestion from the classroom QA video. It works great.

### Valid Trajectories

Generated trajectory meets all the criteria in the rubric:

1. The car is able to drive at least 4.32 miles without incident. 
2. The car doesn't drive faster than the speed limit.
3. The car does not exceed a total acceleration of `10 m/s^2` and a jerk of `10 m/s^3`.
4. Car does not have collisions.
5. The car stays in its lane, except for the time between changing lanes.
6. The car is able to change lanes

You can watch the recorded video on [YouTube](https://youtu.be/o7HwBBmaOyc).

### Implementation Description



## License
[MIT License](https://github.com/mhBahrami/Path_Planning/blob/master/LICENSE).

