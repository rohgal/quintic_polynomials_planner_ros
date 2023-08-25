#include "quintic_polynomials_planner.hpp"

Quintic_Polynomials_Planner::Quintic_Polynomials_Planner() {
  polynomials_server = nh_.advertiseService("get_polynomials", &Quintic_Polynomials_Planner::solve_polynomials, this);
}

Quintic_Polynomials_Planner::~Quintic_Polynomials_Planner() {
  ROS_INFO("Destruct Quintic_Polynomials_Planner");
};

std::vector<std_msgs::Float32> Quintic_Polynomials_Planner::convertToFloat32Array(const std::vector<double>& input) {
  std::vector<std_msgs::Float32> output;
  output.resize(input.size());

  for (size_t i = 0; i < input.size(); ++i) {
    output[i].data = static_cast<float>(input[i]);
  }

  return output;
}

vector<double> Quintic_Polynomials_Planner::get_coefficients(vector<double>& start, vector<double>& end, double T) {
  MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
		
	MatrixXd B = MatrixXd(3,1);	    
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
			    
	MatrixXd Ai = A.inverse();
	
	MatrixXd C = Ai*B;
	
	vector<double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}
	
    return result;
}

double Quintic_Polynomials_Planner::get_point(vector<double>& coefficients, double t) {
  double point;
  point = coefficients[0] + coefficients[1]*t + coefficients[2]*t*t + coefficients[3]*t*t*t + coefficients[4]*t*t*t*t + coefficients[5]*t*t*t*t*t;

  return point;
}

double Quintic_Polynomials_Planner::get_vel(vector<double>& coefficients, double t) {
  double vel;
  vel = coefficients[1] + 2*coefficients[2]*t + 3*coefficients[3]*t*t + 4*coefficients[4]*t*t*t + 5*coefficients[5]*t*t*t*t;

  return vel;
}

double Quintic_Polynomials_Planner::get_acc(vector<double>& coefficients, double t) {
  double acc;
  acc = 2*coefficients[2] + 6*coefficients[3]*t + 12*coefficients[4]*t*t + 20*coefficients[5]*t*t*t;

  return acc;
}

bool Quintic_Polynomials_Planner::solve_polynomials(quintic_polynomials_planner_ros::GetPolynomials::Request& req, quintic_polynomials_planner_ros::GetPolynomials::Response& res) {
  double start_time = ros::Time::now().toSec();

  double start_yaw_rad = req.start_yaw.data * M_PI /180.0;
  double end_yaw_rad = req.goal_yaw.data * M_PI /180.0;
  vector<double> start_x = {req.start_pnt.x, req.start_vel.data * cos(start_yaw_rad), req.start_acc.data * cos(start_yaw_rad)};
  vector<double> end_x = {req.goal_pnt.x, req.goal_vel.data * cos(end_yaw_rad), req.goal_acc.data * cos(end_yaw_rad)};
  vector<double> start_y = {req.start_pnt.y, req.start_vel.data * sin(start_yaw_rad), req.start_acc.data * sin(start_yaw_rad)};
  vector<double> end_y = {req.goal_pnt.y, req.goal_vel.data * sin(end_yaw_rad), req.goal_acc.data * sin(end_yaw_rad)};
  double T = req.T.data;
  double dt = req.dt.data;

  coefficients_x = get_coefficients(start_x, end_x, T);
  coefficients_y = get_coefficients(start_y, end_y, T);

  res.x_coeff = convertToFloat32Array(coefficients_x);
  res.y_coeff = convertToFloat32Array(coefficients_y);

  tf::Quaternion quat;

  for (double t = 0.0; t <= T + dt; t += dt) {
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist vel;
    geometry_msgs::Accel acc;

    double pnt_x = get_point(coefficients_x, t);
    double pnt_y = get_point(coefficients_y, t);
    double vel_x = get_vel(coefficients_x, t);
    double vel_y = get_vel(coefficients_y, t);
    // double acc_x = get_acc(coefficients_x, t);
    // double acc_y = get_acc(coefficients_y, t);

    pnts_x.push_back(pnt_x);
    pnts_y.push_back(pnt_y);
    // vels_x.push_back(vel_x);
    // vels_y.push_back(vel_y);
    // accs_x.push_back(acc_x);
    // accs_y.push_back(acc_y);
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "burger/odom";
    pose.pose.position.x = pnt_x;
    pose.pose.position.y = pnt_y;
    quat.setRPY(0.0, 0.0, atan2(vel_y, vel_x));
    tf::quaternionTFToMsg(quat, pose.pose.orientation);
    // vel.linear.x = vel_x;
    // vel.linear.y = vel_y;
    // acc.linear.x = acc_x;
    // acc.linear.y = acc_y;
    
    res.path.poses.push_back(pose);
    // res.vels.push_back(vel);
    // res.accs.push_back(acc);
    pathspeed.path.poses.push_back(pose);
  }

  double end_time = ros::Time::now().toSec();
  ROS_INFO("Generate a path successfully! Use %f s", end_time - start_time);
  
  return true;
}

