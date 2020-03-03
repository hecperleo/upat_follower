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

#include <ros/package.h>
#include <ros/ros.h>
#include <uav_abstraction_layer/State.h>
#include <uav_abstraction_layer/ual.h>
#include <upat_follower/Visualize.h>
#include <upat_follower/generator.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <ctime>
#include <fstream>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

class Visualization {
   public:
    Visualization();
    ~Visualization();

    bool save_experiment = false;
    uav_abstraction_layer::State ual_state_;
    nav_msgs::Path current_path_, init_path_, generated_path_;
    std::vector<std_msgs::Float32> init_times_;

    std::ofstream csv_normal_distances_, csv_current_path_, csv_generated_waypoints_, csv_init_waypoints_;

    void pubMsgs();
    void saveMissionData();

   private:
    // Callbacks
    void ualStateCallback(const uav_abstraction_layer::State &_ual_state);
    void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
    bool visualCallback(upat_follower::Visualize::Request &_req_visual, upat_follower::Visualize::Response &_res_visual);
    // Methods
    int calculateDistanceOnPath(int _prev_normal_pos_on_path, double _meters, nav_msgs::Path &_path_search);
    int calculateNormalDistance(Eigen::Vector3f &_current_point, double _search_range, int _prev_normal_pos_on_path, nav_msgs::Path &_path_search);
    visualization_msgs::Marker readModel(std::string _model_type);
    bool checkWaypointReached(double _check_distance);
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    ros::Subscriber sub_pose_, sub_state_;
    // Publishers
    ros::Publisher pub_init_path_, pub_generated_path_, pub_current_path_, pub_uav_model_;
    // Services
    ros::ServiceServer server_visualize_;
    // Variables
    geometry_msgs::PoseStamped ual_pose_;
    geometry_msgs::TwistStamped current_vel_, desired_vel_;
    nav_msgs::Path interp1_path_;
    visualization_msgs::Marker uav_model_;
    std::vector<double> normal_dist_generated_path_, normal_dist_init_path_;
    double normal_distance_;
    int prev_normal_pos_on_init_path_ = 0;
    int prev_normal_pos_on_generated_path_ = 0;
    int waypoint_to_check_ = 1;
    std::vector<std_msgs::Float32> generated_times_;
    // Params
    int uav_id_;
    std::string model_, ns_prefix_;
};