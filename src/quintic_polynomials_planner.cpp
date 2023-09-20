#include "quintic_polynomials_planner.hpp"

Quintic_Polynomials_Planner::Quintic_Polynomials_Planner() {
  polynomials_server = nh_.advertiseService("get_polynomials", &Quintic_Polynomials_Planner::solve_polynomials, this);
  pub_polypath_vis = nh_.advertise<nav_msgs::Path>("polynomial_path_vis", 1, true);
  nh_.param<string>("FilePath", file_path, "/home/cai/train_ws/src/2D_trainer/pkgs/quintic_polynomials_planner_ros/path_saved/path.json");
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

void Quintic_Polynomials_Planner::save_path2yaml(const nav_msgs::Path& path_msg, const std::string& file_path)
{
    std::ofstream yaml_file(file_path);

    if (!yaml_file.is_open())
    {
        ROS_ERROR("Failed to open YAML file for writing");
        return;
    }

    yaml_file << std::fixed << std::setprecision(6);
    yaml_file << "path_poses:" << std::endl;

    for (const auto& pose_stamped : path_msg.poses)
    {
        double x = pose_stamped.pose.position.x;
        double y = pose_stamped.pose.position.y;
        double yaw_degrees = calculateYawfromQuat(pose_stamped.pose.orientation);

        yaml_file << "  - { x: " << x
                  << ", y: " << y
                  << ", yaw: deg(" << yaw_degrees << ") }" << std::endl;
    }

    yaml_file.close();

    ROS_INFO("Path data saved as YAML");
}

void Quintic_Polynomials_Planner::save_path2json(const nav_msgs::Path& path_msg, const std::string& file_path)
{
    // Create a JSON object
    json path_json;

    // Serialize header
    path_json["header"]["seq"] = path_msg.header.seq;
    path_json["header"]["stamp"]["secs"] = path_msg.header.stamp.sec;
    path_json["header"]["stamp"]["nsecs"] = path_msg.header.stamp.nsec;
    path_json["header"]["frame_id"] = path_msg.header.frame_id;

    // Serialize poses
    for (const auto& pose_stamped : path_msg.poses)
    {
        json pose_json;

        // Serialize header of pose_stamped
        pose_json["header"]["seq"] = pose_stamped.header.seq;
        pose_json["header"]["stamp"]["secs"] = pose_stamped.header.stamp.sec;
        pose_json["header"]["stamp"]["nsecs"] = pose_stamped.header.stamp.nsec;
        pose_json["header"]["frame_id"] = pose_stamped.header.frame_id;

        // Serialize pose
        pose_json["pose"]["position"]["x"] = pose_stamped.pose.position.x;
        pose_json["pose"]["position"]["y"] = pose_stamped.pose.position.y;
        pose_json["pose"]["position"]["z"] = pose_stamped.pose.position.z;
        pose_json["pose"]["orientation"]["x"] = pose_stamped.pose.orientation.x;
        pose_json["pose"]["orientation"]["y"] = pose_stamped.pose.orientation.y;
        pose_json["pose"]["orientation"]["z"] = pose_stamped.pose.orientation.z;
        pose_json["pose"]["orientation"]["w"] = pose_stamped.pose.orientation.w;

        path_json["poses"].push_back(pose_json);
    }

    // Save JSON data to a file
    std::ofstream json_file(file_path);
    if (json_file.is_open())
    {
        json_file << std::setw(4) << path_json.dump(); // Indent JSON for readability
        json_file.close();
        ROS_INFO("Path data saved as JSON");
    }
    else
    {
        ROS_ERROR("Failed to open JSON file for writing");
    }
}

double Quintic_Polynomials_Planner::calculateYawfromQuat(const geometry_msgs::Quaternion& orientation)
{
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(orientation, tf_quaternion);

    double roll, pitch, yaw;
    tf::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    return yaw * 180.0 / M_PI;
}

geometry_msgs::Quaternion Quintic_Polynomials_Planner::calculateQuatfromYaw(const double& yaw)
{
    tf::Quaternion tf_quaternion;
    geometry_msgs::Quaternion orientation;
    tf_quaternion.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(tf_quaternion, orientation);

    return orientation;
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

  nav_msgs::Path path_vis;

  res.path.header.frame_id = "map";
  path_vis.header.frame_id = "map";

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
    pose.pose.orientation = calculateQuatfromYaw(atan2(vel_y, vel_x));
    // vel.linear.x = vel_x;
    // vel.linear.y = vel_y;
    // acc.linear.x = acc_x;
    // acc.linear.y = acc_y;

    res.path.poses.push_back(pose);
    // res.vels.push_back(vel);
    // res.accs.push_back(acc);
    path_vis.poses.push_back(pose);
  }
  pub_polypath_vis.publish(path_vis);
  save_path2json(path_vis, file_path);

  double end_time = ros::Time::now().toSec();
  ROS_INFO("Generate a path successfully! Use %f s", end_time - start_time);
  
  return true;
}

