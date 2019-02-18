#include <uav_path_manager/visualization.h>

Visualization::Visualization() : nh_(), pnh_("~") {
    // Parameters
    pnh_.getParam("uav_id", uav_id_);
    // Subscriptions
    sub_pose_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/pose", 0, &Visualization::ualPoseCallback, this);
    // Publishers
    pub_init_path_ = nh_.advertise<nav_msgs::Path>("/uav_path_manager/visualization/manager/uav_" + std::to_string(uav_id_) + "/init_path", 1);
    pub_generated_path_ = nh_.advertise<nav_msgs::Path>("/uav_path_manager/visualization/manager/uav_" + std::to_string(uav_id_) + "/generated_path", 1);
    // Services
    server_visualize_ = nh_.advertiseService("/uav_path_manager/visualization/uav_" + std::to_string(uav_id_) + "/visualize", &Visualization::visualCallback, this);
    // Flags
    // Initialize path
    // Save data
}

Visualization::~Visualization() {
}

void Visualization::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose_ = *_ual_pose;
}

bool Visualization::visualCallback(uav_path_manager::Visualize::Request &_req_visual,
                                   uav_path_manager::Visualize::Response &_res_visual) {
    init_path_ = _req_visual.init_path;
    generated_path_ = _req_visual.generated_path;

    return true;
}

void Visualization::pubMsgs() {
    pub_init_path_.publish(init_path_);
    pub_generated_path_.publish(generated_path_);
    // pub_current_path_.publish(current_path_);
}