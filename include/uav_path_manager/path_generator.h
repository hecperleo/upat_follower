#include <ros/ros.h>

#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/GeneratePath.h>
#include <Eigen/Eigen>
#include "ecl/geometry.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"
// linealInterp1
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

class PathGenerator {
   public:
    PathGenerator();
    ~PathGenerator();

    nav_msgs::Path createPathInterp1(std::vector<double> list_x, std::vector<double> list_y, std::vector<double> list_z, int path_size, int new_path_size);
    nav_msgs::Path createPathCubicSpline(std::vector<double> list_x, std::vector<double> list_y, std::vector<double> list_z, int path_size);

   private:
    // Callbacks
    bool pathCallback(uav_path_manager::GeneratePath::Request &req_path, uav_path_manager::GeneratePath::Response &res_path);
    // Methods
    int nearestNeighbourIndex(std::vector<double> &x, double &value);
    std::vector<double> linealInterp1(std::vector<double> &x, std::vector<double> &y, std::vector<double> &x_new);
    std::vector<double> interpWaypointList(std::vector<double> list_pose_axis, int amount_of_points);
    nav_msgs::Path constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z);
    nav_msgs::Path pathManagement(std::vector<double> list_pose_x, std::vector<double> list_pose_y, std::vector<double> list_pose_z);
    // Node handlers
    ros::NodeHandle nh;
    // Services
    ros::ServiceServer srv_generate_path;
    // Variables
    nav_msgs::Path output_path_;
    enum mode_t { mode_interp1,
                  mode_cubic_spline_loyal,
                  mode_cubic_spline,
                  mode_idle };
    mode_t mode = mode_idle;
};