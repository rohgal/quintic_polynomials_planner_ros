# Quintic Polynomials Planner for ROS
This package is an Quintic Polynomials Planner configured with ROS service.

## Service Request
- start_pnt (geometry_msgs/Point)

- start_vel (geometry_msgs/Twist)

- start_acc (geometry_msgs/Twist)

- goal_pnt (geometry_msgs/Point)

- goal_vel (geometry_msgs/Twist)

- goal_acc (geometry_msgs/Twist)

- T (std_msgs/Float32)

- dt (std_msgs/Float32)

## Service Response
- x_coeff (std_msgs/Float32[])

- y_coeff (std_msgs/Float32[])

- path (nav_msgs/Path)

- vels (geometry_msgs/Twist[])

- accs (geometry_msgs/Twist[])

## Prerequisites
- Ubuntu
- ROS

Tested on Ubuntu 20.04, ROS Noetic.
## Getting Started
- Clone this repo in your src directory of ROS workspace.
```
git clone https://github.com/rohgal/astar_ros.git
```
- Build
```
catkin_make
```
- Launch Quintic Polynomials ROS server
```
roslaunch quintic_polynomials_planner_ros quintic_polynomial_server.launch
```
