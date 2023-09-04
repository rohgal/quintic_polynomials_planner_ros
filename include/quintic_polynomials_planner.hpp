#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseStamped.h>
#include <plan2control_msgs/PathSpeed.h>
#include <Eigen/Eigen>
#include "quintic_polynomials_planner_ros/GetPolynomials.h"

using namespace std;
using Eigen::MatrixXd;

class Quintic_Polynomials_Planner {
  public:
    Quintic_Polynomials_Planner();
    ~Quintic_Polynomials_Planner();

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer polynomials_server;
    ros::Publisher pub_polypath_vis;
    
    vector<double> coefficients_x;
    vector<double> coefficients_y;
    vector<double> pnts_x;
    vector<double> pnts_y;
    vector<double> vels_x;
    vector<double> vels_y;
    vector<double> accs_x;
    vector<double> accs_y;

    std::vector<std_msgs::Float32> convertToFloat32Array(const std::vector<double>& input);
    vector<double> get_coefficients(vector<double>& start, vector<double>& end, double T);
    double get_point(vector<double>& coefficients, double t);
    double get_vel(vector<double>& coefficients, double t);
    double get_acc(vector<double>& coefficients, double t);
    bool solve_polynomials(quintic_polynomials_planner_ros::GetPolynomials::Request& req, quintic_polynomials_planner_ros::GetPolynomials::Response& res);
};