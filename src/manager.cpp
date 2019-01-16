#include <path_generator_follower/manager.h>

Manager::Manager() {
    nh = ros::NodeHandle();
    // Subscriptions
    sub_pose = nh.subscribe("/uav_1/ual/pose", 0, &Manager::ualPoseCallback, this);
    sub_state = nh.subscribe("/uav_1/ual/state", 0, &Manager::ualStateCallback, this);
    sub_path = nh.subscribe("output_path", 0, &Manager::pathCallback, this);
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

void Manager::runMission() {
    uav_abstraction_layer::TakeOff take_off;
    uav_abstraction_layer::Land land;
    switch (ual_state.state) {
        case 2:  // Landed armed
            take_off.request.height = 5.0;
            take_off.request.blocking = true;
            srvTakeOff.call(take_off);
            break;
        case 3:  // Taking of
            // cout taking of?
            break;
        case 4:  // Flying auto
            // If uav is not on start point, go to it
            // If uav on path, follow it
            /* If uav on path end, call land srv
                land.request.blocking = true;
                srvLand.call(land);
                */
            break;
        case 5:  // Landing
            // cout landing?
            break;
        default:
            break;
    }
}