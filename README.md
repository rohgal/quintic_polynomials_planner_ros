# Quintic Polynomials Planner for ROS
This repo is an quintic polynomials planner package compatible with ROS.

## Service Request
- start_pnt (geometry_msgs/Point)

- start_vel (std_msgs/Float32)

- start_acc (std_msgs/Float32)

- start_yaw (std_msgs/Float32)

- goal_pnt (geometry_msgs/Point)

- goal_vel (std_msgs/Float32)

- goal_acc (std_msgs/Float32)

- goal_yaw (std_msgs/Float32)

- T (std_msgs/Float32)

- dt (std_msgs/Float32)

## Service Response
- x_coeff (std_msgs/Float32[])

- y_coeff (std_msgs/Float32[])

- path (nav_msgs/Path)

- vels (geometry_msgs/Twist[])

- accs (geometry_msgs/Twist[])

## Topic to Be Published
- /polynomial_path_vis (nav_msgs/Path)

## Generated Path is Saved as YAML or JSON
to **~/path_saved/path.yaml** or **~/path_saved/path.json**

## Prerequisites
- Ubuntu
- ROS
- Eigen

Tested on Ubuntu 20.04, ROS Noetic.
## Getting Started
- Clone this repo in your src directory of ROS workspace.
```
git clone https://github.com/rohgal/quintic_polynomials_planner_ros.git
```
- Build
```
catkin_make
```
- Launch quintic polynomials ROS server.
```
roslaunch quintic_polynomials_planner_ros quintic_polynomial_server.launch
```
