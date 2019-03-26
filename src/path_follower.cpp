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

#include <uav_path_manager/path_follower.h>

PathFollower::PathFollower() : nh_(), pnh_("~") {
    // Parameters
    pnh_.getParam("uav_id", uav_id_);
    pnh_.getParam("debug", debug_);
    // Subscriptions
    sub_pose_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/pose", 0, &PathFollower::ualPoseCallback, this);
    // Publishers
    pub_output_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/output_vel", 1000);
    if (debug_) {
        std::cout << "Debug true!" << std::endl;
        pub_point_look_ahead_ = nh_.advertise<geometry_msgs::PointStamped>("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/debug_point_look_ahead", 1000);
        pub_point_normal_ = nh_.advertise<geometry_msgs::PointStamped>("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/debug_point_normal", 1000);
        pub_point_search_normal_begin_ = nh_.advertise<geometry_msgs::PointStamped>("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/debug_point_search_begin", 1000);
        pub_point_search_normal_end_ = nh_.advertise<geometry_msgs::PointStamped>("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/debug_point_search_end", 1000);
    }
    // Services
    server_follow_path_ = nh_.advertiseService("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/follow_path", &PathFollower::pathCallback, this);
}

PathFollower::~PathFollower() {
}

bool PathFollower::pathCallback(uav_path_manager::FollowPath::Request &_req_path,
                                uav_path_manager::FollowPath::Response &_res_path) {
    target_path_ = _req_path.generated_path;
    target_vel_path_ = _req_path.generated_path_vel_percentage;
    target_vel_path_.header.frame_id = _req_path.generated_path.header.frame_id;
    follower_mode_ = _req_path.follower_mode.data;
    switch (follower_mode_) {
        case 1:  // Path
            look_ahead_ = _req_path.look_ahead.data;
            cruising_speed_ = _req_path.cruising_speed.data;
            break;
        case 2:  // Trajectory
            for (int i = 0; i < _req_path.generated_max_vel_percentage.size(); i++) {
                generated_max_vel_percentage_.push_back(_req_path.generated_max_vel_percentage.at(i).data);
            }
            max_vel_ = _req_path.max_velocity.data;
            break;
    }
    _res_path.ok.data = true;
    // flag_run_ = false;

    return true;
}

void PathFollower::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose_ = *_ual_pose;
}

int PathFollower::calculatePosOnPath(Eigen::Vector3f _current_point, double _search_range, int _prev_normal_pos_on_path, nav_msgs::Path _path_search) {
    std::vector<double> vec_distances;
    // const int search_range = 10;  // Find UAV position on path in a range, do not search in all path.
    // int start_search_pos_on_path = _prev_normal_pos_on_path - _search_range;
    // int end_search_pos_on_path = _prev_normal_pos_on_path + _search_range;
    int start_search_pos_on_path = calculateDistanceOnPath(_prev_normal_pos_on_path, -_search_range);
    int end_search_pos_on_path = calculateDistanceOnPath(_prev_normal_pos_on_path, _search_range);
    // if (start_search_pos_on_path <= 0) start_search_pos_on_path = 0;
    // if (end_search_pos_on_path >= target_path_.poses.size()) end_search_pos_on_path = target_path_.poses.size() - 1;
    for (int i = start_search_pos_on_path; i < end_search_pos_on_path; i++) {
        Eigen::Vector3f target_path_point;
        target_path_point = Eigen::Vector3f(target_path_.poses.at(i).pose.position.x, target_path_.poses.at(i).pose.position.y, target_path_.poses.at(i).pose.position.z);
        vec_distances.push_back((target_path_point - _current_point).norm());
    }
    auto smallest_distance = std::min_element(vec_distances.begin(), vec_distances.end());
    int pos_on_path = smallest_distance - vec_distances.begin();

    return pos_on_path + start_search_pos_on_path;
}

int PathFollower::calculatePosLookAhead(int _pos_on_path) {
    int pos_look_ahead;
    std::vector<double> vec_distances;
    double temp_dist = 0.0;
    for (_pos_on_path; _pos_on_path < target_path_.poses.size() - 1; _pos_on_path++) {
        Eigen::Vector3f p1 = Eigen::Vector3f(target_path_.poses.at(_pos_on_path).pose.position.x, target_path_.poses.at(_pos_on_path).pose.position.y, target_path_.poses.at(_pos_on_path).pose.position.z);
        Eigen::Vector3f p2 = Eigen::Vector3f(target_path_.poses.at(_pos_on_path + 1).pose.position.x, target_path_.poses.at(_pos_on_path + 1).pose.position.y, target_path_.poses.at(_pos_on_path + 1).pose.position.z);
        temp_dist = temp_dist + (p2 - p1).norm();
        if (temp_dist < look_ahead_) {
            pos_look_ahead = _pos_on_path;
        } else {
            _pos_on_path = target_path_.poses.size();
        }
    }

    return pos_look_ahead;
}

double PathFollower::changeLookAhead(int _pos_on_path) {
    // ROS_WARN("la: %f, max: %f, %: %f", max_vel_ * generated_max_vel_percentage_[_pos_on_path], max_vel_, generated_max_vel_percentage_[_pos_on_path]);
    return max_vel_ * generated_max_vel_percentage_[_pos_on_path];
}

geometry_msgs::TwistStamped PathFollower::calculateVelocity(Eigen::Vector3f _current_point, int _pos_look_ahead) {
    geometry_msgs::TwistStamped out_vel;
    Eigen::Vector3f target_p, unit_vec, hypo_vec;
    target_p = Eigen::Vector3f(target_path_.poses.at(_pos_look_ahead).pose.position.x, target_path_.poses.at(_pos_look_ahead).pose.position.y, target_path_.poses.at(_pos_look_ahead).pose.position.z);
    double distance = (target_p - _current_point).norm();
    switch (follower_mode_) {
        case 1:
            unit_vec = (target_p - _current_point) / distance;
            unit_vec = unit_vec / unit_vec.norm();
            out_vel.twist.linear.x = unit_vec(0) * cruising_speed_;
            out_vel.twist.linear.y = unit_vec(1) * cruising_speed_;
            out_vel.twist.linear.z = unit_vec(2) * cruising_speed_;
            break;
        case 2:
            hypo_vec = (target_p - _current_point);
            out_vel.twist.linear.x = hypo_vec(0);
            out_vel.twist.linear.y = hypo_vec(1);
            out_vel.twist.linear.z = hypo_vec(2);
            // unit_vec = (target_p - _current_point) / distance;
            // unit_vec = unit_vec / unit_vec.norm();
            // out_vel.twist.linear.x = unit_vec(0) * cruising_speed_;
            // out_vel.twist.linear.y = unit_vec(1) * cruising_speed_;
            // out_vel.twist.linear.z = unit_vec(2) * cruising_speed_;
            break;
    }
    out_vel.header.frame_id = target_path_.header.frame_id;

    return out_vel;
}

int PathFollower::calculateDistanceOnPath(int _prev_normal_pos_on_path, double _meters) {
    int pos_equals_dist;
    double dist_to_front, dist_to_back, temp_dist;
    std::vector<double> vec_distances;
    Eigen::Vector3f p_prev = Eigen::Vector3f(target_path_.poses.at(_prev_normal_pos_on_path).pose.position.x, target_path_.poses.at(_prev_normal_pos_on_path).pose.position.y, target_path_.poses.at(_prev_normal_pos_on_path).pose.position.z);
    Eigen::Vector3f p_front = Eigen::Vector3f(target_path_.poses.front().pose.position.x, target_path_.poses.front().pose.position.y, target_path_.poses.front().pose.position.z);
    Eigen::Vector3f p_back = Eigen::Vector3f(target_path_.poses.back().pose.position.x, target_path_.poses.back().pose.position.y, target_path_.poses.back().pose.position.z);
    dist_to_front = (p_prev - p_front).norm();
    dist_to_back = (p_prev - p_back).norm();
    temp_dist = 0.0;
    if (_meters > 0) {
        if (_meters < dist_to_back) {
            for (int i = _prev_normal_pos_on_path; i < target_path_.poses.size() - 1; i++) {
                Eigen::Vector3f p1 = Eigen::Vector3f(target_path_.poses.at(i).pose.position.x, target_path_.poses.at(i).pose.position.y, target_path_.poses.at(i).pose.position.z);
                Eigen::Vector3f p2 = Eigen::Vector3f(target_path_.poses.at(i + 1).pose.position.x, target_path_.poses.at(i + 1).pose.position.y, target_path_.poses.at(i + 1).pose.position.z);
                temp_dist = temp_dist + (p2 - p1).norm();
                if (temp_dist < _meters) {
                    pos_equals_dist = i;
                } else {
                    i = target_path_.poses.size();
                }
            }
        } else {
            pos_equals_dist = target_path_.poses.size() - 1;
        }
    } else {
        if (_meters < dist_to_front) {
            pos_equals_dist = 0;
            for (int i = _prev_normal_pos_on_path; i >= 1; i--) {
                Eigen::Vector3f p1 = Eigen::Vector3f(target_path_.poses.at(i).pose.position.x, target_path_.poses.at(i).pose.position.y, target_path_.poses.at(i).pose.position.z);
                Eigen::Vector3f p0 = Eigen::Vector3f(target_path_.poses.at(i - 1).pose.position.x, target_path_.poses.at(i - 1).pose.position.y, target_path_.poses.at(i - 1).pose.position.z);
                temp_dist = temp_dist + (p1 - p0).norm();
                if (temp_dist < fabs(_meters/2)) {
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

void PathFollower::prepareDebug(double _search_range, int _normal_pos_on_path, int _pos_look_ahead) {
    point_normal_.header.frame_id = point_look_ahead_.header.frame_id =
        point_search_normal_begin_.header.frame_id = point_search_normal_end_.header.frame_id =
            target_path_.header.frame_id;
    point_normal_.point = target_path_.poses.at(_normal_pos_on_path).pose.position;
    point_look_ahead_.point = target_path_.poses.at(_pos_look_ahead).pose.position;
    int start_search_pos_on_path = calculateDistanceOnPath(prev_normal_pos_on_path_, -_search_range);
    int end_search_pos_on_path = calculateDistanceOnPath(prev_normal_pos_on_path_, _search_range);
    point_search_normal_begin_.point = target_path_.poses.at(start_search_pos_on_path).pose.position;
    point_search_normal_end_.point = target_path_.poses.at(end_search_pos_on_path).pose.position;
}

void PathFollower::pubMsgs() {
    pub_output_velocity_.publish(out_velocity_);
    if (debug_) {
        pub_point_look_ahead_.publish(point_look_ahead_);
        pub_point_normal_.publish(point_normal_);
        pub_point_search_normal_begin_.publish(point_search_normal_begin_);
        pub_point_search_normal_end_.publish(point_search_normal_end_);
    }
}

void PathFollower::followPath() {
    if (target_path_.poses.size() > 1) {
        Eigen::Vector3f current_point, target_path0_point;
        current_point = Eigen::Vector3f(ual_pose_.pose.position.x, ual_pose_.pose.position.y, ual_pose_.pose.position.z);
        target_path0_point = Eigen::Vector3f(target_path_.poses.at(0).pose.position.x, target_path_.poses.at(0).pose.position.y, target_path_.poses.at(0).pose.position.z);
        if ((current_point - target_path0_point).norm() < 1) {
            flag_run_ = true;
        }
        if (flag_run_) {
            double search_range_normal_pos = look_ahead_*1.5;
            int normal_pos_on_path = calculatePosOnPath(current_point, search_range_normal_pos, prev_normal_pos_on_path_, target_path_);
            if (follower_mode_ == 2) {
                int const search_range_vel = target_vel_path_.poses.size() * 0.4;
                int normal_vel_on_path = calculatePosOnPath(current_point, search_range_vel, prev_normal_vel_on_path_, target_vel_path_);
                prev_normal_vel_on_path_ = normal_vel_on_path;
                look_ahead_ = changeLookAhead(normal_vel_on_path);
            }
            int pos_look_ahead = calculatePosLookAhead(normal_pos_on_path);
            out_velocity_ = calculateVelocity(current_point, pos_look_ahead);
            if (debug_) {
                prepareDebug(search_range_normal_pos, normal_pos_on_path, pos_look_ahead);
            }
            prev_normal_pos_on_path_ = normal_pos_on_path;
        }
    }
}
