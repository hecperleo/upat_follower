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
#ifndef GENERATOR_H
#define GENERATOR_H

#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <ros/ros.h>
#include <upat_follower/GeneratePath.h>
#include <upat_follower/GenerateTrajectory.h>
#include <Eigen/Eigen>
#include "ecl/geometry.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32.h"

namespace upat_follower {

class Generator {
   public:
    Generator();
    Generator(double _vxy, double _vz_up, double _vz_dn, bool _debug = false);
    ~Generator();

    double max_velocity_;
    nav_msgs::Path out_path_;
    std::vector<double> generated_times_;
    nav_msgs::Path generateTrajectory(nav_msgs::Path &_init_path, std::vector<double> &_times, int _generator_mode = 0);
    nav_msgs::Path generatePath(nav_msgs::Path &_init_path, int _generator_mode = 0, double _final_points = 1000);
    std::vector<double> interpWaypointList(std::vector<double> &_list_pose_axis, int _amount_of_points);

   private:
    // Callbacks
    bool generatePathCb(upat_follower::GeneratePath::Request &_req_path, upat_follower::GeneratePath::Response &_res_path);
    bool generateTrajectoryCb(upat_follower::GenerateTrajectory::Request &_req_trajectory, upat_follower::GenerateTrajectory::Response &_res_trajectory);
    // Methods
    double checkSmallestMaxVel();
    double updateParam(const std::string &_param_id);
    int nearestNeighbourIndex(std::vector<double> &_x, double &_value);
    std::vector<double> linealInterp1(std::vector<double> &_x, std::vector<double> &_y, std::vector<double> &_x_new);
    nav_msgs::Path constructPath(std::vector<double> &_wps_x, std::vector<double> &_wps_y, std::vector<double> &_wps_z);
    nav_msgs::Path pathManagement(std::vector<double> &_list_pose_x, std::vector<double> &_list_pose_y, std::vector<double> &_list_pose_z);
    nav_msgs::Path createPathCubicSpline(std::vector<double> &_list_x, std::vector<double> &_list_y, std::vector<double> &_list_z, int _path_size);
    nav_msgs::Path createPathSmoothSpline(std::vector<double> &_list_x, std::vector<double> &_list_y, std::vector<double> &_list_z, int _path_size);
    nav_msgs::Path createPathInterp1(std::vector<double> &_list_x, std::vector<double> &_list_y, std::vector<double> &_list_z, int _path_size, int _new_path_size);
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Services
    ros::ServiceClient get_param_client_;
    ros::ServiceServer server_generate_path_, server_generate_trajectory_;
    // Variables
    int interp1_final_size_ = 10000;
    enum mode_t { mode_interp1_,
                  mode_smooth_spline_,
                  mode_cubic_spline_,
                  mode_trajectory_,
                  mode_idle_ };
    mode_t mode_ = mode_idle_;
    // Params
    bool debug_ = false;
    std::map<std::string, double> mavros_params_;
};

}  // namespace upat_follower

#endif /* GENERATOR_H */