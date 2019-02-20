#include <uav_path_manager/path_follower.h>
#include <handy_tools/pid_controller.h>
#include <tf2/utils.h>     // to convert quaternion to roll-pitch-yaw



PathFollower::PathFollower() : nh_(), pnh_("~") {
    // Parameters
    pnh_.getParam("uav_id", uav_id_);
    // Subscriptions
    sub_pose_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/pose", 0, &PathFollower::ualPoseCallback, this);
    // Publishers
    pub_output_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/output_vel", 1000);
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
    flag_run_ = false;

    return true;
}

void PathFollower::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose_ = *_ual_pose;
    // get yaw
     tf2::Quaternion q3(_ual_pose->pose.orientation.x,
                       _ual_pose->pose.orientation.y,
                       _ual_pose->pose.orientation.z,
                       _ual_pose->pose.orientation.w);
    q3.normalize();     //Avoids a warning
    current_yaw_ = tf2::getYaw(q3);
}

float PathFollower::calculateYawDiff(float _desired_yaw, float _current_yaw) {
    float yaw_diff = _desired_yaw - _current_yaw;
    while (yaw_diff < -M_PI) yaw_diff += 2*M_PI;
    while (yaw_diff >  M_PI) yaw_diff -= 2*M_PI;

    return yaw_diff;
}


int PathFollower::calculatePosOnPath(Eigen::Vector3f _current_point, int _search_range, int _prev_normal_pos_on_path, nav_msgs::Path _path_search) {
    std::vector<double> vec_distances;
    // const int search_range = 10;  // Find UAV position on path in a range, do not search in all path.
    int start_search_pos_on_path = _prev_normal_pos_on_path - _search_range;
    int end_search_pos_on_path = _prev_normal_pos_on_path + _search_range;
    if (start_search_pos_on_path <= 0) start_search_pos_on_path = 0;
    if (end_search_pos_on_path >= target_path_.poses.size()) end_search_pos_on_path = target_path_.poses.size() - 1;
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
    float desired_yaw = atan2(target_path_.poses.at(_pos_look_ahead).pose.position.y-_current_point.y(), target_path_.poses.at(_pos_look_ahead).pose.position.x-_current_point.x());
    float yaw_diff = calculateYawDiff(desired_yaw, current_yaw_);
    float yaw_pid_p = 0.4;
    float yaw_pid_i = 0.02;
    float yaw_pid_d = 0.0;
    float sampling_period = 0.01;
    grvc::utils::PidController yaw_pid("yaw", yaw_pid_p, yaw_pid_i, yaw_pid_d);
    float commanded_yaw_rate = yaw_pid.control_signal(yaw_diff, sampling_period);
    switch (follower_mode_) {
        case 1:
            unit_vec = (target_p - _current_point) / distance;
            unit_vec = unit_vec / unit_vec.norm();
            out_vel.twist.linear.x = unit_vec(0) * cruising_speed_;
            out_vel.twist.linear.y = unit_vec(1) * cruising_speed_;
            out_vel.twist.linear.z = unit_vec(2) * cruising_speed_;
            out_vel.twist.angular.z = commanded_yaw_rate;

            break;
        case 2:
            hypo_vec = (target_p - _current_point);
            out_vel.twist.linear.x = hypo_vec(0);
            out_vel.twist.linear.y = hypo_vec(1);
            out_vel.twist.linear.z = hypo_vec(2);
            out_vel.twist.angular.z = commanded_yaw_rate;

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

void PathFollower::pubMsgs() {
    pub_output_velocity_.publish(out_velocity_);
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
            int const search_range_pos = 10;
            int normal_pos_on_path = calculatePosOnPath(current_point, search_range_pos, prev_normal_pos_on_path_, target_path_);
            if (follower_mode_ == 2) {
                int const search_range_vel = target_vel_path_.poses.size() * 0.4;
                int normal_vel_on_path = calculatePosOnPath(current_point, search_range_vel, prev_normal_vel_on_path_, target_vel_path_);
                prev_normal_vel_on_path_ = normal_vel_on_path;
                look_ahead_ = changeLookAhead(normal_vel_on_path);
            }
            int pos_look_ahead = calculatePosLookAhead(normal_pos_on_path);
            out_velocity_ = calculateVelocity(current_point, pos_look_ahead);
            prev_normal_pos_on_path_ = normal_pos_on_path;
        }
    }
}