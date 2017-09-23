# CarND-Path-Planning-Project

Goals of Project:
-	Design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. 
-	A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.


## Rubric Points
Here I will consider the rubric points individually and describe how I addressed each point in my implementation. 

---
### Compilation

Code must compile without errors with cmake and make.

The `CMakeLists.txt` is moved to `src` directory. To start the compation, use the following line:
```
cmake ../src
```
---
### Valid Trajectories

- In every iteration, the car's target speed (`target_speed`) value will be set to speed limit (`speed_limit` is slightly less than the exact value). If another vehicle is detected in front of the car and none of the adjacent lanes are free, the target speed will become the front vehicle's speed.
Therefore, the car would never drive faster than the speed limit. Moreover, it will be always checked that, unless obstructed by traffic (`too_close`), the car won't driving much slower than speed limit (line 612 in `main.cpp`)

- Since the reference speed keeps incrementing with `0.1 m/s` steps, max Acceleration and Jerk are not exceeded and a total acceleration is always less than 10 m/s^2 and a jerk less than 10 m/s^3.

- If there is a vehicle in front, the car would slow down to avoid collision, also it wouldn't perform a lane change unless the adjacent lane is free. Therefore, car does not have collisions and it will not come into contact with any of the other cars on the road.

- Since __Fernet coordinates__ and __spline__  are used, the generated trajectory would always have the same value of `d` to make sure the car stays in its lane, except for the time between changing lanes.

- The lane change happens based on the look ahead horizon wich is 1 second (50 points each 20ms appart), and the transition is smooth to control jerk and acceleration. Thus, the car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.


- Lanes _552_ to _600_ of `main.cpp` are checking for empty adjacent lanes when the car get stuck behind a slower vehicle. If there is an empty lane, the car will change lanes.

- The function `isFree` is in fact checking if the adjucent lane is clear for a lane change. TO this end, the car is just looking for a safe gap between two vehicles to merge in. 

- Putting all of the above code together, the car can now drive at least 4.32 miles without incident.
---
### Reflection
you have already read it!

Note: the suggestion section has not been implemented yet.
