#include <ros/ros.h>

#include <mavros_msgs/ParamGet.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/GeneratePath.h>
#include <Eigen/Eigen>
#include "ecl/geometry.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
// linealInterp1
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

class PathGenerator {
   public:
    PathGenerator();
    ~PathGenerator();

    nav_msgs::Path createPathInterp1(std::vector<double> _list_x, std::vector<double> _list_y, std::vector<double> _list_z, int _path_size, int _new_path_size);
    nav_msgs::Path createPathCubicSpline(std::vector<double> _list_x, std::vector<double> _list_y, std::vector<double> _list_z, int _path_size);
    nav_msgs::Path createTrajectory(std::vector<double> _list_x, std::vector<double> _list_y, std::vector<double> _list_z, int _path_size, std::vector<double> _max_vel_percentage);
    enum mode_t { mode_interp1_,
                  mode_cubic_spline_loyal_,
                  mode_cubic_spline_,
                  mode_trajectory_,
                  mode_idle_ };
    mode_t mode_ = mode_idle_;

   private:
    // Callbacks
    bool pathCallback(uav_path_manager::GeneratePath::Request &_req_path, uav_path_manager::GeneratePath::Response &_res_path);
    // Methods
    int nearestNeighbourIndex(std::vector<double> &_x, double &_value);
    std::vector<double> linealInterp1(std::vector<double> &_x, std::vector<double> &_y, std::vector<double> &_x_new);
    std::vector<double> interpWaypointList(std::vector<double> _list_pose_axis, int _amount_of_points);
    nav_msgs::Path constructPath(std::vector<double> _wps_x, std::vector<double> _wps_y, std::vector<double> _wps_z);
    nav_msgs::Path pathManagement(std::vector<double> _list_pose_x, std::vector<double> _list_pose_y, std::vector<double> _list_pose_z);
    double checkSmallestMaxVel();
    double updateParam(const std::string &_param_id);
    // Node handlers
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    // Services
    ros::ServiceServer server_generate_path_, srv_generate_trajectory_;
    ros::ServiceClient get_param_client_;
    std::map<std::string, double> mavros_params_;
    // Variables
    double smallest_max_vel_ = 1.0;
    int size_vec_percentage_ = 0;
    int interp1_final_size_ = 10000;
};