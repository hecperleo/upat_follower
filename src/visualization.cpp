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

#include <uav_path_manager/visualization.h>

Visualization::Visualization() : nh_(), pnh_("~") {
    // Parameters
    pnh_.getParam("uav_id", uav_id_);
    std::string robot_model;
    pnh_.getParam("robot_model", robot_model);
    // Subscriptions
    sub_pose_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/pose", 0, &Visualization::ualPoseCallback, this);
    // Publishers
    pub_init_path_ = nh_.advertise<nav_msgs::Path>("/uav_path_manager/visualization/uav_" + std::to_string(uav_id_) + "/init_path", 1);
    pub_generated_path_ = nh_.advertise<nav_msgs::Path>("/uav_path_manager/visualization/uav_" + std::to_string(uav_id_) + "/generated_path", 1);
    pub_current_path_ = nh_.advertise<nav_msgs::Path>("/uav_path_manager/visualization/uav_" + std::to_string(uav_id_) + "/current_path", 1);
    pub_uav_model_ = nh_.advertise<visualization_msgs::Marker>("/uav_path_manager/visualization/uav_" + std::to_string(uav_id_) + "/uav_model", 1);
    // Services
    server_visualize_ = nh_.advertiseService("/uav_path_manager/visualization/uav_" + std::to_string(uav_id_) + "/visualize", &Visualization::visualCallback, this);
    // Flags
    // Initialize path
    // Save data

    uav_model_ = readModel(robot_model);
}

Visualization::~Visualization() {
}

bool Visualization::visualCallback(uav_path_manager::Visualize::Request &_req_visual,
                                   uav_path_manager::Visualize::Response &_res_visual) {
    init_path_ = _req_visual.init_path;
    uav_model_.header.frame_id = init_path_.header.frame_id;
    generated_path_ = _req_visual.generated_path;
    current_path_ = _req_visual.current_path;

    return true;
}

void Visualization::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose_ = *_ual_pose;
}

visualization_msgs::Marker Visualization::readModel(std::string _model) {
    visualization_msgs::Marker model_;
    model_.id = uav_id_;
    model_.type = visualization_msgs::Marker::MESH_RESOURCE;
    model_.mesh_resource = "package://robots_description/models/" + _model + "/meshes/multirotor.dae";
    model_.action = visualization_msgs::Marker::ADD;
    model_.color.a = 1;
    if (_model == "mbzirc") {
        model_.scale.x = 0.001;
        model_.scale.y = 0.001;
        model_.scale.z = 0.001;
    }
    if (_model == "iris") {
        model_.scale.x = 1;
        model_.scale.y = 1;
        model_.scale.z = 1;
    }
    model_.mesh_use_embedded_materials = true;
    switch (uav_id_) {
        case 1:
            model_.color.r = 0.0;
            model_.color.g = 1.0;
            model_.color.b = 1.0;
            break;
        case 2:
            model_.color.r = 0.541;
            model_.color.g = 1.0;
            model_.color.b = 0.019;
            break;
    }

    return model_;
}

void Visualization::pubMsgs() {
    pub_init_path_.publish(init_path_);
    pub_generated_path_.publish(generated_path_);
    pub_current_path_.publish(current_path_);
    uav_model_.pose = ual_pose_.pose;
    pub_uav_model_.publish(uav_model_);
}