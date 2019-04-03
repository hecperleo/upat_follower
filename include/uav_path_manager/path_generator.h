//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 Hector Perez Leon
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#ifndef PATHGENERATOR_H
#define PATHGENERATOR_H

#include <mavros_msgs/ParamGet.h>
#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/GeneratePath.h>
#include <uav_path_manager/PrepareTrajectory.h>
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

namespace uav_path_manager {

class PathGenerator {
   public:
    PathGenerator();
    PathGenerator(double _vxy, double _vz_up, double _vz_dn, bool _debug = false);
    ~PathGenerator();

    double max_velocity_;
    nav_msgs::Path out_path_;
    nav_msgs::Path generated_path_vel_percentage_;
    std::vector<double> generated_max_vel_percentage_;
    nav_msgs::Path generateTrajectory(nav_msgs::Path _init_path, std::vector<double> _max_vel_percentage);
    nav_msgs::Path generatePath(nav_msgs::Path _init_path, int _generator_mode = 0);
    // For tests (should be private)
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
    bool generatePathCb(uav_path_manager::GeneratePath::Request &_req_path, uav_path_manager::GeneratePath::Response &_res_path);
    bool generateTrajectoryCb(uav_path_manager::PrepareTrajectory::Request &_req_trajectory, uav_path_manager::PrepareTrajectory::Response &_res_trajectory);
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
    ros::ServiceServer server_generate_path_, server_generate_trajectory_;
    ros::ServiceClient get_param_client_;
    std::map<std::string, double> mavros_params_;
    // Variables
    double smallest_max_vel_ = 1.0;
    int size_vec_percentage_ = 0;
    int interp1_final_size_ = 10000;
    // Params
    bool debug_;
};

}  // namespace uav_path_manager

#endif /* PATHGENERATOR_H */