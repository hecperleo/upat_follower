#include <uav_path_manager/path_follower.h>

PathFollower::PathFollower(): nh_(), pnh_("~") {
    // Parameters
    pnh_.getParam("uav_id", uav_id_);
    // Subscriptions
    sub_pose_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/pose", 0, &PathFollower::ualPoseCallback, this);
    // Publishers
    pub_output_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/output_vel", 1000);
    // Services
    srv_get_generated_path_ = nh_.advertiseService("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/generated_path", &PathFollower::pathCallback, this);
}

PathFollower::~PathFollower() {
}

bool PathFollower::pathCallback(uav_path_manager::GetGeneratedPath::Request &_req_path,
                                uav_path_manager::GetGeneratedPath::Response &_res_path) {
    target_path_ = _req_path.generated_path;
    _res_path.ok.data = true;
    look_ahead_ = _req_path.look_ahead.data;
    cruising_speed_ = _req_path.cruising_speed.data;
    flag_run_ = false;
    return true;
}

void PathFollower::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose_ = *_ual_pose;
}

int PathFollower::calculatePosOnPath(Eigen::Vector3f _current_p) {
    std::vector<double> vec_distances;
    for (int i = 0; i < target_path_.poses.size() - 1; i++) {
        Eigen::Vector3f target_path_point;
        target_path_point = Eigen::Vector3f(target_path_.poses.at(i).pose.position.x, target_path_.poses.at(i).pose.position.y, target_path_.poses.at(i).pose.position.z);
        vec_distances.push_back((target_path_point - _current_p).norm());
    }
    auto smallest_distance = std::min_element(vec_distances.begin(), vec_distances.end());
    int pos_on_path = smallest_distance - vec_distances.begin();

    return pos_on_path;
}

int PathFollower::calculatePosLookAhead(int _pos_on_path) {
    int pos_la;
    std::vector<double> vec_distances;
    Eigen::Vector3f target_path_point;
    target_path_point = Eigen::Vector3f(target_path_.poses.at(_pos_on_path).pose.position.x, target_path_.poses.at(_pos_on_path).pose.position.y, target_path_.poses.at(_pos_on_path).pose.position.z);
    for (_pos_on_path; _pos_on_path < target_path_.poses.size(); _pos_on_path++) {
        Eigen::Vector3f look_ahead_point = Eigen::Vector3f(target_path_.poses.at(_pos_on_path).pose.position.x, target_path_.poses.at(_pos_on_path).pose.position.y, target_path_.poses.at(_pos_on_path).pose.position.z);
        if((look_ahead_point - target_path_point).norm() < look_ahead_){
            pos_la = _pos_on_path;
        }else{
            _pos_on_path = target_path_.poses.size();
        }
    }

    return pos_la;
}

geometry_msgs::TwistStamped PathFollower::calculateVelocity(Eigen::Vector3f _current_p, int _pos_la) {
    geometry_msgs::TwistStamped out_vel;
    Eigen::Vector3f target_p;
    target_p = Eigen::Vector3f(target_path_.poses.at(_pos_la).pose.position.x, target_path_.poses.at(_pos_la).pose.position.y, target_path_.poses.at(_pos_la).pose.position.z);
    double distance = (target_p - _current_p).norm();
    Eigen::Vector3f unit_vec = (target_p - _current_p) / distance;
    unit_vec = unit_vec / unit_vec.norm();
    out_vel.twist.linear.x = unit_vec(0) * cruising_speed_;
    out_vel.twist.linear.y = unit_vec(1) * cruising_speed_;
    out_vel.twist.linear.z = unit_vec(2) * cruising_speed_;
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
            int normal_pos_on_path = calculatePosOnPath(current_point);
            int pos_look_ahead = calculatePosLookAhead(normal_pos_on_path);
            out_velocity_ = calculateVelocity(current_point, pos_look_ahead);
        }
    } else {
    }
}