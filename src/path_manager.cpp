#include <path_generator_follower/path_manager.h>

Manager::Manager() {
    nh = ros::NodeHandle();
    // Subscriptions
    sub_pose = nh.subscribe("/uav_1/ual/pose", 0, &Manager::ualPoseCallback, this);
    sub_state = nh.subscribe("/uav_1/ual/state", 0, &Manager::ualStateCallback, this);
    sub_path = nh.subscribe("/generator/output_path", 0, &Manager::pathCallback, this);
    sub_velocity = nh.subscribe("/follower/output_vel", 0, &Manager::velocityCallback, this);
    // Publishers
    pub_init_path = nh.advertise<nav_msgs::Path>("/manager/init_path", 1000);
    pub_generator_mode = nh.advertise<std_msgs::Int8>("/manager/generator_mode", 1000);
    pub_set_pose = nh.advertise<geometry_msgs::PoseStamped>("/uav_1/ual/set_pose", 1000);
    pub_set_velocity = nh.advertise<geometry_msgs::TwistStamped>("/uav_1/ual/set_velocity", 1000);
    // Services
    srvTakeOff = nh.serviceClient<uav_abstraction_layer::TakeOff>("/uav_1/ual/take_off");
    srvLand = nh.serviceClient<uav_abstraction_layer::Land>("/uav_1/ual/land");

    on_path = false;
    end_path = false;

    init_path = constructPath(list_init_x, list_init_y, list_init_z);
    generator_mode.data = 2;
}

Manager::~Manager() {
}

nav_msgs::Path Manager::constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z) {
    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses(wps_x.size());
    path_msg.header.frame_id = "uav_1_home";
    for (int i = 0; i < wps_x.size(); i++) {
        poses.at(i).pose.position.x = wps_x[i];
        poses.at(i).pose.position.y = wps_y[i];
        poses.at(i).pose.position.z = wps_z[i];
        poses.at(i).pose.orientation.x = 0;
        poses.at(i).pose.orientation.y = 0;
        poses.at(i).pose.orientation.z = 0;
        poses.at(i).pose.orientation.w = 1;
    }
    path_msg.poses = poses;
    return path_msg;
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

void Manager::pubMsgs(){
    pub_init_path.publish(init_path);    
    pub_generator_mode.publish(generator_mode);
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
                if (!end_path) {
                    if (!on_path) {
                        if ((current_p - path0_p).norm() > 0.5) {
                            pub_set_pose.publish(path.poses.at(0));
                        } else if (0.2 > (current_p - path0_p).norm()) {
                            pub_set_pose.publish(path.poses.front());
                            on_path = true;
                        }
                    } else {
                        if (1.0 > (current_p - path_end_p).norm()) {
                            pub_set_pose.publish(path.poses.back());
                            on_path = false;
                            end_path = true;
                        } else {
                            pub_set_velocity.publish(velocity_);
                        }
                    }
                } else {
                    if (1.0 > (current_p - path_end_p).norm() && (current_p - path_end_p).norm() > 0.2) {
                        std::cout << "dist to end: " << (current_p - path_end_p).norm() << " end: " << end_path << " on: " << on_path << std::endl;
                        pub_set_pose.publish(path.poses.back());
                    } else {
                        land.request.blocking = true;
                        srvLand.call(land);
                    }
                }
                break;
            case 5:  // Landing
                break;
            default:
                break;
        }
    }
}