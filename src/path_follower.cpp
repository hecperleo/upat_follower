#include <uav_path_manager/path_follower.h>

PathFollower::PathFollower() {
    // Node Handle
    nh = ros::NodeHandle();
    pnh = ros::NodeHandle("~");
    // Parameters
    pnh.getParam("uav_id", uav_id);
    // Subscriptions
    sub_pose = nh.subscribe("/uav_" + std::to_string(uav_id) + "/ual/pose", 0, &PathFollower::ualPoseCallback, this);
    // Publishers
    pub_output_vel = nh.advertise<geometry_msgs::TwistStamped>("/uav_path_manager/follower/output_vel", 1000);
    // Services
    srv_get_generated_path = nh.advertiseService("/uav_path_manager/manager/generated_path", &PathFollower::pathCallback, this);
}

PathFollower::~PathFollower() {
}

bool PathFollower::pathCallback(uav_path_manager::GetGeneratedPath::Request &req_path,
                                uav_path_manager::GetGeneratedPath::Response &res_path) {
    path = req_path.generated_path;
    flag_run = false;
    return true;
}

void PathFollower::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose = *_ual_pose;
}

int PathFollower::calculatePosOnPath(Eigen::Vector3f current_p) {
    std::vector<double> vec_distances;
    for (int i = 0; i < path.poses.size() - 1; i++) {
        Eigen::Vector3f path_p;
        path_p = Eigen::Vector3f(path.poses.at(i).pose.position.x, path.poses.at(i).pose.position.y, path.poses.at(i).pose.position.z);
        vec_distances.push_back((path_p - current_p).norm());
    }
    auto smallest_dist = std::min_element(vec_distances.begin(), vec_distances.end());
    int pos_on_path = smallest_dist - vec_distances.begin();

    return pos_on_path;
}

int PathFollower::calculatePosLookAhead(int pos_on_path) {
    int pos_la;
    std::vector<double> vec_distances;
    Eigen::Vector3f path_p;
    path_p = Eigen::Vector3f(path.poses.at(pos_on_path).pose.position.x, path.poses.at(pos_on_path).pose.position.y, path.poses.at(pos_on_path).pose.position.z);
    for (int i = pos_on_path; i < path.poses.size() - 1; i++) {
        Eigen::Vector3f la_p;
        la_p = Eigen::Vector3f(path.poses.at(i).pose.position.x, path.poses.at(i).pose.position.y, path.poses.at(i).pose.position.z);
        vec_distances.push_back((la_p - path_p).norm());
    }
    std::vector<double>::iterator up = std::lower_bound(vec_distances.begin(), vec_distances.end(), look_ahead);
    pos_la = up - vec_distances.begin();

    return pos_la;
}

geometry_msgs::TwistStamped PathFollower::calculateVelocity(Eigen::Vector3f current_p, int pos_la) {
    geometry_msgs::TwistStamped out_vel;
    double cruising_speed = 1.0;
    Eigen::Vector3f target_p;
    target_p = Eigen::Vector3f(path.poses.at(pos_la).pose.position.x, path.poses.at(pos_la).pose.position.y, path.poses.at(pos_la).pose.position.z);
    double distance = (target_p - current_p).norm();
    Eigen::Vector3f unit_vec = (target_p - current_p) / distance;
    unit_vec = unit_vec / unit_vec.norm();
    out_vel.twist.linear.x = unit_vec(0) * cruising_speed;
    out_vel.twist.linear.y = unit_vec(1) * cruising_speed;
    out_vel.twist.linear.z = unit_vec(2) * cruising_speed;
    out_vel.header.frame_id = path.header.frame_id;

    return out_vel;
}

void PathFollower::pubMsgs() {
    pub_output_vel.publish(out_velocity);
}

void PathFollower::followPath() {
    if (path.poses.size() > 1) {
        Eigen::Vector3f current_point, path0_point;
        current_point = Eigen::Vector3f(ual_pose.pose.position.x, ual_pose.pose.position.y, ual_pose.pose.position.z);
        path0_point = Eigen::Vector3f(path.poses.at(0).pose.position.x, path.poses.at(0).pose.position.y, path.poses.at(0).pose.position.z);
        if ((current_point - path0_point).norm() < 1) {
            flag_run = true;
        }
        if (flag_run) {
            int normal_pos_on_path = calculatePosOnPath(current_point);
            int pos_look_ahead = calculatePosLookAhead(normal_pos_on_path);
            out_velocity = calculateVelocity(current_point, normal_pos_on_path + pos_look_ahead);
        }
    } else {
    }
}