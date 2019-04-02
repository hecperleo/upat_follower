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

#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/PreparePath.h>
#include <uav_path_manager/PrepareTrajectory.h>
#include <uav_path_manager/GeneratePath.h>
// #include <uav_path_manager/GenerateTrajectory.h>
#include <uav_path_manager/path_generator.h>
#include <Eigen/Eigen>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"

class PathFollower {
   public:
    PathFollower();
    PathFollower(int _uav_id, double _vxy = 2.0, double _vz_up = 3.0, double _vz_dn = 1.0);
    ~PathFollower();

    void pubMsgs();
    geometry_msgs::TwistStamped out_velocity_;
    geometry_msgs::TwistStamped getVelocity();
    void updatePose(const geometry_msgs::PoseStamped &_ual_pose);
    nav_msgs::Path prepareTrajectory(nav_msgs::Path _init_path, std::vector<double> _max_vel_percentage);
    nav_msgs::Path preparePath(nav_msgs::Path _init_path, int _generator_mode = 0, double _look_ahead = 1.2, double _cruising_speed = 1.0);

   private:
    // Callbacks
    void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
    bool preparePathCb(uav_path_manager::PreparePath::Request &_req_path, uav_path_manager::PreparePath::Response &_res_path);
    bool prepareTrajectoryCb(uav_path_manager::PrepareTrajectory::Request &_req_trajectory, uav_path_manager::PrepareTrajectory::Response &_res_trajectory);
    // Methods
    double changeLookAhead(int _pos_on_path);
    int calculatePosLookAhead(int _pos_on_path);
    int calculateDistanceOnPath(int _prev_normal_pos_on_path, double _meters);
    int calculatePosOnPath(Eigen::Vector3f _current_point, double _search_range, int _prev_normal_pos_on_path, nav_msgs::Path _path_search);
    void prepareDebug(double _search_range, int _normal_pos_on_path, int _pos_look_ahead);
    geometry_msgs::TwistStamped calculateVelocity(Eigen::Vector3f _current_p, int _pos_la);
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    ros::Subscriber sub_pose_;
    // Publishers
    ros::Publisher pub_output_velocity_, pub_point_look_ahead_, pub_point_normal_, pub_point_search_normal_begin_, pub_point_search_normal_end_;
    // Services
    ros::ServiceServer server_prepare_path_, server_prepare_trajectory_;
    // Variables
    int follower_mode_;
    int prev_normal_pos_on_path_ = 0;
    int prev_normal_vel_on_path_ = 0;
    bool flag_run_ = false;
    double look_ahead_ = 1.0;
    double cruising_speed_ = 1.0;
    double max_vel_ = 1.0;
    geometry_msgs::PoseStamped ual_pose_;
    nav_msgs::Path target_path_, target_vel_path_;
    std::vector<double> generated_max_vel_percentage_;
    // Params
    int uav_id_;
    bool debug_;
    double vxy_ = 2.0;
    double vz_up_ = 3.0;
    double vz_dn_ = 1.0;
    // Debug
    geometry_msgs::PointStamped point_look_ahead_, point_normal_, point_search_normal_begin_, point_search_normal_end_;
};

#endif /* PATHFOLLOWER_H */