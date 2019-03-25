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

#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/FollowPath.h>
#include <Eigen/Eigen>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"

class PathFollower {
   public:
    PathFollower();
    ~PathFollower();

    void pubMsgs();
    void followPath();

   private:
    // Callbacks
    bool pathCallback(uav_path_manager::FollowPath::Request &_req_path, uav_path_manager::FollowPath::Response &_res_path);
    void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
    // Methods
    double changeLookAhead(int _pos_on_path);
    int calculatePosOnPath(Eigen::Vector3f _current_point, int _search_range, int _prev_normal_pos_on_path, nav_msgs::Path _path_search);
    int calculatePosLookAhead(int _pos_on_path);
    geometry_msgs::TwistStamped calculateVelocity(Eigen::Vector3f _current_p, int _pos_la);
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    ros::Subscriber sub_pose_;
    // Publishers
    ros::Publisher pub_output_velocity_;
    // Services
    ros::ServiceServer server_follow_path_;
    // Variables
    int uav_id_, follower_mode_;
    bool flag_run_ = false;
    double look_ahead_ = 1.0;
    double cruising_speed_ = 1.0;
    double max_vel_ = 1.0;
    int prev_normal_pos_on_path_ = 0;
    int prev_normal_vel_on_path_ = 0;
    nav_msgs::Path target_path_, target_vel_path_;
    geometry_msgs::PoseStamped ual_pose_;
    geometry_msgs::TwistStamped out_velocity_;
    std::vector<double> generated_max_vel_percentage_;
};