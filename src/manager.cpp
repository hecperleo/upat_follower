#include <path_generator_follower/manager.h>

Manager::Manager() {
    nh = ros::NodeHandle();
    // Subscriptions
    sub_pose = nh.subscribe("/uav_1/ual/pose", 0, &Manager::ualPoseCallback, this);
    sub_state = nh.subscribe("/uav_1/ual/state", 0, &Manager::ualStateCallback, this);
    sub_path = nh.subscribe("output_path", 0, &Manager::pathCallback, this);
    sub_velocity = nh.subscribe("output_vel", 0, &Manager::velocityCallback, this);
    // Publishers
    pub_set_pose = nh.advertise<geometry_msgs::PoseStamped>("/uav_1/ual/set_pose", 1000);
    pub_set_velocity = nh.advertise<geometry_msgs::TwistStamped>("/uav_1/ual/set_velocity", 1000);
    // Services
    srvTakeOff = nh.serviceClient<uav_abstraction_layer::TakeOff>("/uav_1/ual/take_off");
    srvLand = nh.serviceClient<uav_abstraction_layer::Land>("/uav_1/ual/land");
}

Manager::~Manager() {
}

void Manager::pathCallback(const nav_msgs::Path &_path) {
    if (_path.poses.size() > 1) {
        path = _path;
    }
}

void Manager::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose = *_ual_pose;
}

void Manager::ualStateCallback(const uav_abstraction_layer::State &_ual_state) {
    ual_state.state = _ual_state.state;
}

void Manager::velocityCallback(const geometry_msgs::TwistStamped &_velocity) {
    velocity_ = _velocity;
}

void Manager::runMission() {
    if (path.poses.size() > 1) {
        uav_abstraction_layer::TakeOff take_off;
        uav_abstraction_layer::Land land;
        Eigen::Vector3f current_p, path0_p, path_end_p;
        current_p = Eigen::Vector3f(ual_pose.pose.position.x, ual_pose.pose.position.y, ual_pose.pose.position.z);
        path0_p = Eigen::Vector3f(path.poses.front().pose.position.x, path.poses.front().pose.position.y, path.poses.front().pose.position.z);
        path_end_p = Eigen::Vector3f(path.poses.back().pose.position.x, path.poses.back().pose.position.y, path.poses.back().pose.position.z);
        switch (ual_state.state) {
            case 2:  // Landed armed
                take_off.request.height = 5.0;
                take_off.request.blocking = true;
                srvTakeOff.call(take_off);
                break;
            case 3:  // Taking of
                break;
            case 4:  // Flying auto
                if ((current_p - path0_p).norm() > 0.5 && !on_path) {
                    pub_set_pose.publish(path.poses.at(0));
                } else if ((current_p - path0_p).norm() < 0.2 && !on_path) {
                    pub_set_pose.publish(path.poses.front());
                    on_path = true;
                } else if (on_path) {
                    pub_set_velocity.publish(velocity_);
                } else if ((current_p - path_end_p).norm() < 1.0 && on_path) {
                    pub_set_pose.publish(path.poses.back());
                    on_path = false;
                } else if ((current_p - path_end_p).norm() < 0.2 && !on_path) {
                    land.request.blocking = true;
                    srvLand.call(land);
                } else {
                }
                // If uav is not on start point, go to it
                // If uav on path, follow it
                // If uav on path end, call land srv
                break;
            case 5:  // Landing
                // cout landing?
                break;
            default:
                break;
        }
    }
}