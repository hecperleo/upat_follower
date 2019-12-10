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

#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include <upat_follower/PreparePath.h>
#include <upat_follower/PrepareTrajectory.h>
#include <upat_follower/UpdatePath.h>
#include <upat_follower/UpdateTrajectory.h>
#include <upat_follower/generator.h>
#include <Eigen/Eigen>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"

namespace upat_follower {

class Follower {
   public:
    Follower();
    Follower(int _uav_id, bool _debug = false);
    ~Follower();

    void pubMsgs();
    geometry_msgs::TwistStamped out_velocity_;
    geometry_msgs::TwistStamped getVelocity();
    void updatePose(const geometry_msgs::PoseStamped &_ual_pose);
    void updatePath(nav_msgs::Path _new_target_path);
    void updateTrajectory(nav_msgs::Path _new_target_path, nav_msgs::Path _new_target_vel_path);
    nav_msgs::Path prepareTrajectory(nav_msgs::Path _init_path, std::vector<double> _times);
    nav_msgs::Path preparePath(nav_msgs::Path _init_path, int _generator_mode = 0, double _look_ahead = 1.2, double _cruising_speed = 1.0);
    int position_on_path_ = 0;

   private:
    // Callbacks
    void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
    bool preparePathCb(upat_follower::PreparePath::Request &_req_path, upat_follower::PreparePath::Response &_res_path);
    bool prepareTrajectoryCb(upat_follower::PrepareTrajectory::Request &_req_trajectory, upat_follower::PrepareTrajectory::Response &_res_trajectory);
    bool updatePathCb(upat_follower::UpdatePath::Request &_req_path, upat_follower::UpdatePath::Response &_res_path);
    bool updateTrajectoryCb(upat_follower::UpdateTrajectory::Request &_req_trajectory, upat_follower::UpdateTrajectory::Response &_res_trajectory);
    // Methods
    void capMaxVelocities();
    double changeLookAhead(int _pos_on_path);
    int calculatePosLookAhead(int _pos_on_path);
    int calculateDistanceOnPath(int _prev_normal_pos_on_path, double _meters);
    int calculatePosOnPath(Eigen::Vector3f _current_point, double _search_range, int _prev_normal_pos_on_path, nav_msgs::Path _path_search);
    void prepareDebug(double _search_range, int _normal_pos_on_path, int _pos_look_ahead, int _prev_normal);
    geometry_msgs::TwistStamped calculateVelocity(Eigen::Vector3f _current_point, int _pos_look_ahead, int _pos_on_path = 0);
    std::vector<double> timesToMaxVelPercentage(nav_msgs::Path _init_path, std::vector<double> _times);
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    ros::Subscriber sub_pose_;
    // Publishers
    ros::Publisher pub_output_velocity_, pub_point_look_ahead_, pub_point_normal_, pub_point_search_normal_begin_, pub_point_search_normal_end_;
    // Services
    ros::ServiceServer server_prepare_path_, server_prepare_trajectory_;
    // Variables
    double vxy_ = 1.0;
    double vz_up_ = 1.0;
    double vz_dn_ = 1.0;
    double smallest_max_velocity_;
    std::vector<double> mpc_xy_vel_max_ = {0.0, 20.0};   // Default PX4 parameter limits
    std::vector<double> mpc_z_vel_max_up_ = {0.5, 8.0};  // Default PX4 parameter limits
    std::vector<double> mpc_z_vel_max_dn_ = {0.5, 4.0};  // Default PX4 parameter limits
    int follower_mode_;
    int prev_normal_pos_on_path_ = 0;
    int prev_normal_vel_on_path_ = 0;
    int increase_vel_count_ = 1;
    geometry_msgs::PoseStamped ual_pose_;
    nav_msgs::Path target_path_, target_vel_path_;
    double look_ahead_, cruising_speed_, max_vel_, increase_vel_;
    std::vector<double> generated_times_;
    // Params
    int uav_id_;
    bool debug_ = false;
    bool debug_class_ = false;
    std::string ns_prefix_ = "uav_";
    // Debug
    geometry_msgs::PointStamped point_look_ahead_, point_normal_, point_search_normal_begin_, point_search_normal_end_;
};

}  // namespace upat_follower

#endif /* FOLLOWER_H */