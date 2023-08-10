#include "quintic_polynomials_planner.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "quintic_polynomials_node");
  Quintic_Polynomials_Planner planner;
  ROS_INFO("Quintic Polynomials Planner is ready.");
  ros::spin();
  return 0;
};