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
#include <uav_abstraction_layer/Land.h>
#include <uav_abstraction_layer/State.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/FollowPath.h>
#include <uav_path_manager/GeneratePath.h>
#include <uav_path_manager/Visualize.h>
#include <Eigen/Eigen>
#include <fstream>
#include "ecl/geometry.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"

class PathManager {
   public:
    PathManager();
    ~PathManager();

    void runMission();
    void callVisualization();

   private:
    // Callbacks
    void ualStateCallback(const uav_abstraction_layer::State &_ual_state);
    void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
    void velocityCallback(const geometry_msgs::TwistStamped &_velocity);
    // Methods
    nav_msgs::Path csvToPath(std::string _file_name);
    nav_msgs::Path constructPath(std::vector<double> _wps_x, std::vector<double> _wps_y, std::vector<double> _wps_z, std::string frame_id);
    void saveDataForTesting();
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    ros::Subscriber sub_pose_, sub_state_, sub_velocity_;
    // Publishers
    ros::Publisher pub_set_velocity_, pub_set_pose_;
    // Services
    ros::ServiceClient client_take_off_, client_land_, client_generate_path_, client_follow_path_, client_visualize_;
    // Variables
    std::string folder_data_name_;
    bool on_path_, end_path_;
    nav_msgs::Path path, vel_percentage_path_, init_path_, current_path_;
    geometry_msgs::PoseStamped ual_pose_;
    geometry_msgs::TwistStamped velocity_;
    uav_abstraction_layer::State ual_state_;
    std::vector<double> max_vel_percentage = {1.0, 0.8, 0.5, 0.8, 1, 0.8, 0.5, 0.8};
    // Params
    int uav_id_;
    bool save_csv_;
    bool trajectory_;
};