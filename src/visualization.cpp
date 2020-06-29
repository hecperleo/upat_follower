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

#include <upat_follower/visualization.h>

Visualization::Visualization() : nh_(), pnh_("~") {
    // Parameters
    pnh_.getParam("uav_id", uav_id_);
    pnh_.getParam("ns_prefix", ns_prefix_);
    std::string robot_model;
    pnh_.getParam("robot_model", robot_model);
    // Subscriptions
    sub_pose_ = nh_.subscribe("/" + ns_prefix_ + std::to_string(uav_id_) + "/ual/pose", 0, &Visualization::ualPoseCallback, this);
    sub_state_ = nh_.subscribe("/" + ns_prefix_ + std::to_string(uav_id_) + "/ual/state", 0, &Visualization::ualStateCallback, this);
    // Publishers
    pub_init_path_ = nh_.advertise<nav_msgs::Path>("/" + ns_prefix_ + std::to_string(uav_id_) + "/upat_follower/visualization/init_path", 1);
    pub_generated_path_ = nh_.advertise<nav_msgs::Path>("/" + ns_prefix_ + std::to_string(uav_id_) + "/upat_follower/visualization/generated_path", 1);
    pub_current_path_ = nh_.advertise<nav_msgs::Path>("/" + ns_prefix_ + std::to_string(uav_id_) + "/upat_follower/visualization/current_path", 1);
    pub_uav_model_ = nh_.advertise<visualization_msgs::Marker>("/" + ns_prefix_ + std::to_string(uav_id_) + "/upat_follower/visualization/uav_model", 1);
    // Services
    server_visualize_ = nh_.advertiseService("/" + ns_prefix_ + std::to_string(uav_id_) + "/upat_follower/visualization/visualize", &Visualization::visualCallback, this);

    // frame id initialization to avoid rviz warnings
    current_path_.header.frame_id = "map";
    generated_path_.header.frame_id = "map";
    init_path_.header.frame_id = "map";
    
    uav_model_ = readModel(robot_model);
}

Visualization::~Visualization() {
}

bool Visualization::visualCallback(upat_follower::Visualize::Request &_req_visual,
                                   upat_follower::Visualize::Response &_res_visual) {
    init_path_ = _req_visual.init_path;
    uav_model_.header.frame_id = init_path_.header.frame_id;
    generated_path_ = _req_visual.generated_path;
    generated_times_ = _req_visual.generated_times;
    init_times_ = _req_visual.init_times;
    current_path_ = _req_visual.current_path;
    current_vel_ = _req_visual.current_vel;
    desired_vel_ = _req_visual.desired_vel;

    return true;
}

void Visualization::ualStateCallback(const uav_abstraction_layer::State &_ual_state) {
    ual_state_.state = _ual_state.state;
}

void Visualization::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose_ = *_ual_pose;
}

visualization_msgs::Marker Visualization::readModel(std::string _model) {
    visualization_msgs::Marker model_;
    
    model_.header.frame_id = "map";
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

int Visualization::calculateNormalDistance(Eigen::Vector3f &_current_point, double _search_range, int _prev_normal_pos_on_path, nav_msgs::Path &_path_search) {
    std::vector<double> vec_distances;
    int start_search_pos_on_path = calculateDistanceOnPath(_prev_normal_pos_on_path, -_search_range, _path_search);
    int end_search_pos_on_path = calculateDistanceOnPath(_prev_normal_pos_on_path, _search_range, _path_search);
    for (int i = start_search_pos_on_path; i < end_search_pos_on_path; i++) {
        Eigen::Vector3f target_path_point;
        target_path_point = Eigen::Vector3f(_path_search.poses.at(i).pose.position.x, _path_search.poses.at(i).pose.position.y, _path_search.poses.at(i).pose.position.z);
        vec_distances.push_back((target_path_point - _current_point).norm());
    }
    auto smallest_distance = std::min_element(vec_distances.begin(), vec_distances.end());
    int pos_on_path = smallest_distance - vec_distances.begin();
    normal_distance_ = *smallest_distance;

    return pos_on_path + start_search_pos_on_path;
}

int Visualization::calculateDistanceOnPath(int _prev_normal_pos_on_path, double _meters, nav_msgs::Path &_path_search) {
    int pos_equals_dist;
    double dist_to_front, dist_to_back, temp_dist;
    std::vector<double> vec_distances;
    Eigen::Vector3f p_prev = Eigen::Vector3f(_path_search.poses.at(_prev_normal_pos_on_path).pose.position.x, _path_search.poses.at(_prev_normal_pos_on_path).pose.position.y, _path_search.poses.at(_prev_normal_pos_on_path).pose.position.z);
    Eigen::Vector3f p_front = Eigen::Vector3f(_path_search.poses.front().pose.position.x, _path_search.poses.front().pose.position.y, _path_search.poses.front().pose.position.z);
    Eigen::Vector3f p_back = Eigen::Vector3f(_path_search.poses.back().pose.position.x, _path_search.poses.back().pose.position.y, _path_search.poses.back().pose.position.z);
    dist_to_front = (p_prev - p_front).norm();
    dist_to_back = (p_prev - p_back).norm();
    temp_dist = 0.0;
    if (_meters > 0) {
        if (_meters < dist_to_back) {
            for (int i = _prev_normal_pos_on_path; i < _path_search.poses.size() - 1; i++) {
                Eigen::Vector3f p1 = Eigen::Vector3f(_path_search.poses.at(i).pose.position.x, _path_search.poses.at(i).pose.position.y, _path_search.poses.at(i).pose.position.z);
                Eigen::Vector3f p2 = Eigen::Vector3f(_path_search.poses.at(i + 1).pose.position.x, _path_search.poses.at(i + 1).pose.position.y, _path_search.poses.at(i + 1).pose.position.z);
                temp_dist = temp_dist + (p2 - p1).norm();
                if (temp_dist < _meters) {
                    pos_equals_dist = i;
                } else {
                    i = _path_search.poses.size();
                }
            }
        } else {
            pos_equals_dist = _path_search.poses.size() - 1;
        }
    } else {
        if (_meters < dist_to_front) {
            pos_equals_dist = 0;
            for (int i = _prev_normal_pos_on_path; i >= 1; i--) {
                Eigen::Vector3f p1 = Eigen::Vector3f(_path_search.poses.at(i).pose.position.x, _path_search.poses.at(i).pose.position.y, _path_search.poses.at(i).pose.position.z);
                Eigen::Vector3f p0 = Eigen::Vector3f(_path_search.poses.at(i - 1).pose.position.x, _path_search.poses.at(i - 1).pose.position.y, _path_search.poses.at(i - 1).pose.position.z);
                temp_dist = temp_dist + (p1 - p0).norm();
                if (temp_dist < fabs(_meters / 2)) {
                    pos_equals_dist = i;
                } else {
                    i = 0;
                }
            }
        } else {
            pos_equals_dist = 0;
        }
    }

    return pos_equals_dist;
}

bool Visualization::checkWaypointReached(double _check_distance) {
    Eigen::Vector3f current_p = Eigen::Vector3f(ual_pose_.pose.position.x, ual_pose_.pose.position.y, ual_pose_.pose.position.z);
    Eigen::Vector3f targ_wp_p = Eigen::Vector3f(init_path_.poses.at(waypoint_to_check_).pose.position.x, init_path_.poses.at(waypoint_to_check_).pose.position.y, init_path_.poses.at(waypoint_to_check_).pose.position.z);
    if ((targ_wp_p - current_p).norm() < _check_distance) {
        waypoint_to_check_++;
        return true;
    } else {
        return false;
    }
}

void Visualization::saveMissionData() {
    static double begin = ros::Time::now().toSec();
    static bool flag_once = true;
    if (flag_once) {
        csv_normal_distances_ << std::fixed << std::setprecision(5);
        upat_follower::Generator generator(1.0, 1.0, 1.0, 0);
        interp1_path_ = generator.generatePath(init_path_, 0);
        flag_once = false;
    }
    Eigen::Vector3f current_point = Eigen::Vector3f(ual_pose_.pose.position.x, ual_pose_.pose.position.y, ual_pose_.pose.position.z);
    csv_normal_distances_ << ros::Time::now().toSec() - begin << ", ";
    if (generated_path_.poses.size() > 1) {
        int normal_pos_on_generated_path = calculateNormalDistance(current_point, 2.0, prev_normal_pos_on_generated_path_, generated_path_);
        normal_dist_generated_path_.push_back(normal_distance_);
        prev_normal_pos_on_generated_path_ = normal_pos_on_generated_path;
        if (generated_times_.size() == generated_path_.poses.size()) {
            csv_normal_distances_ << generated_times_.at(normal_pos_on_generated_path).data << ", ";
        } else {
            csv_normal_distances_ << 0.0 << ", ";
        }
        csv_normal_distances_ << normal_distance_ << ", ";
    }
    if (interp1_path_.poses.size() > 1) {
        int normal_pos_on_init_path = calculateNormalDistance(current_point, 2.0, prev_normal_pos_on_init_path_, interp1_path_);
        normal_dist_init_path_.push_back(normal_distance_);
        // csv_normal_distances_ << normal_distance_ << std::endl;
        csv_normal_distances_ << normal_distance_ << ", ";
        prev_normal_pos_on_init_path_ = normal_pos_on_init_path;
    }

    csv_normal_distances_ << ual_pose_.pose.position.x << ", " << ual_pose_.pose.position.y << ", " << ual_pose_.pose.position.z << ", "
                          << current_vel_.twist.linear.x << ", " << current_vel_.twist.linear.y << ", " << current_vel_.twist.linear.z << ", "
                          << desired_vel_.twist.linear.x << ", " << desired_vel_.twist.linear.y << ", " << desired_vel_.twist.linear.z << std::endl;
}

void Visualization::pubMsgs() {
    pub_init_path_.publish(init_path_);
    pub_generated_path_.publish(generated_path_);
    pub_current_path_.publish(current_path_);
    uav_model_.pose = ual_pose_.pose;
    pub_uav_model_.publish(uav_model_);
}