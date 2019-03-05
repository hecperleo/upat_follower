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
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/Visualize.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

class Visualization {
   public:
    Visualization();
    ~Visualization();

    void pubMsgs();

   private:
    // Callbacks
    void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
    bool visualCallback(uav_path_manager::Visualize::Request &_req_visual, uav_path_manager::Visualize::Response &_res_visual);
    // Methods
    visualization_msgs::Marker readModel(std::string _model_type);
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    ros::Subscriber sub_pose_;
    // Publishers
    ros::Publisher pub_init_path_, pub_generated_path_, pub_current_path_, pub_uav_model_;
    // Services
    ros::ServiceServer server_visualize_;
    // Variables
    geometry_msgs::PoseStamped ual_pose_;
    nav_msgs::Path init_path_, generated_path_, current_path_;
    visualization_msgs::Marker uav_model_;
    // Params
    int uav_id_;
    std::string model_;
};