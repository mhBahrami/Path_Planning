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

Everything has been implemented in [`.\src\main.cpp`](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp).

#### Helper and Constant Variables

I defined some variables to help for a much cleaner implementation. 

> You can find them from [line 13](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L13) to [line 20](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L20).

#### Helper Functions

I defined the following functions and used them in `main()` function:

1. `mphToMps()`:  
   - A function to convert velocity from **mile per hour (mph)** to **meter per second (m/sec)**.
   - [Line 26](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L26).
2. `mpsToMph()`:
   - A function to convert velocity from **meter per second (m/sec)** to **mile per hour (mph)**.
   - [Line 27](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L27).
3. `getD()`:
   - Gets the index of the lane and returns a Frenet `d` value representing the middle of that lane.
   - From [Line 173](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L173) to [Line 175](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L175).
4. `getLane()`:
   - Gets a Frenet `d` value and returns the lane index of it. If it is not a valid `d` it returns `-1`.
   - From [Line 177](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L177) to [Line 192](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L192).
5. `analyzeSensorFusionData()`:
   - It gets `sensor_fusion` vector, driving car `lane` index, the car Frenet `s` value (`car_s`), and the count of unseen waypoints of previous trajectory.
   - It analyzes all sensor fusion data and returns a boolean vector of 3 values indicating that is there any car in the car's left lane, car's lane, and car's right lane (`= true`) or not (`= false`).
   - of course if you are in the very right lane, since there is no right lane for this lane, then `isThereAnyCarOnTheRightAndAhead = false` and if you are in the very left lane then `isThereAnyCarOnTheLeftAndAhead = false`.
   - From [Line 194](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L194) to [Line 237](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L237).
6. `getBehaviour()`:
   - It gets the return boolean vector of `analyzeSensorFusionData()`, a reference to current lane index (`lane`), and a reference to car's velocity value (`ref_vel`).
   - <u>*IF THERE IS A CAR AHEAD ON THE ROAD*</u>, (`isThereAnyCarAhead == true`) the car can does 3 things (From [Line 247](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L247) to [Line 293](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L293)):
     1. **Keep Lane**: if it is impossible to change the lane to the right or the left.
     2. **Change Lane to Left**: changing the lane to the left has higher priority than changing the lane to the right. So, if the car can change the lane on both lanes the code will chose the right lane over the right lane.
     3. **Change Lane to Right**: if car cannot change its lane to left then try to change it to right (if possible).
   - If in any step the code changes the lane we need to reset the `stayInLaneCheckPoint` value to `0.0`. It helps the car stays in its new line a while with the same speed ([Line 245](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L245)). As a result, the car will keep driving in the new line for a while with the same speed. It helps to avoid high jerk or high acceleration or even collision.
   - <u>*IF THERE IS NO CAR AHEAD ON THE ROAD*</u>, only adjust the speed to get to the maximum speed limit. The preference here is to come back to the center lane if possible (from [Line 304](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L304) to [Line 310](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L310)). It gives the car more freedom to change the its lane (to the right and to the left) and reduces the possibility of being stocked in traffic.
   - At the end check the adjusted velocity to not to exceed the maximum speed limit ([Line 313](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L313)).

#### Initializing the Variables

Some variables need to be initialized (From [Line 355](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L355) to [Line 361](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L361)).

#### Implementing the Path Planning and Generating the Best Trajectory

>  You can find the implementation from [Line 401](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L401) to [Line 521](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L521).

- **First**, analyze and predict the behavior by determining the target lane and velocity (from [Line 402](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L402) to [Line 412](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L412)). 
- **Second**, generate the safe trajectory to drive (from [Line 417](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L417) to [Line 519](https://github.com/mhBahrami/Path_Planning/blob/master/src/main.cpp#L519)).
  - Here, I used `spline.h` as described [above](https://github.com/mhBahrami/Path_Planning#splineh).

## License
[MIT License](https://github.com/mhBahrami/Path_Planning/blob/master/LICENSE).

