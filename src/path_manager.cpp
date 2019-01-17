#include <uav_path_manager/path_manager.h>

PathManager::PathManager() {
    nh = ros::NodeHandle();
    // Subscriptions
    sub_pose = nh.subscribe("/uav_1/ual/pose", 0, &PathManager::ualPoseCallback, this);
    sub_state = nh.subscribe("/uav_1/ual/state", 0, &PathManager::ualStateCallback, this);
    // sub_path = nh.subscribe("/generator/output_path", 0, &PathManager::pathCallback, this);
    sub_velocity = nh.subscribe("/uav_path_manager/follower/output_vel", 0, &PathManager::velocityCallback, this);
    // Publishers
    pub_init_path = nh.advertise<nav_msgs::Path>("/uav_path_manager/visualization/manager/init_path", 1000);
    pub_generated_path = nh.advertise<nav_msgs::Path>("/uav_path_manager/visualization/manager/generated_path", 1000);
    pub_set_pose = nh.advertise<geometry_msgs::PoseStamped>("/uav_1/ual/set_pose", 1000);
    pub_set_velocity = nh.advertise<geometry_msgs::TwistStamped>("/uav_1/ual/set_velocity", 1000);
    // Services
    srv_take_off = nh.serviceClient<uav_abstraction_layer::TakeOff>("/uav_1/ual/take_off");
    srv_land = nh.serviceClient<uav_abstraction_layer::Land>("/uav_1/ual/land");
    srv_generated_path = nh.serviceClient<uav_path_manager::GeneratePath>("/uav_path_manager/generator/generate_path");
    srv_give_generated_path = nh.advertiseService("/uav_path_manager/manager/generated_path", &PathManager::pathCallback, this);

    on_path = false;
    end_path = false;

    init_path = constructPath(list_init_x, list_init_y, list_init_z);
}

PathManager::~PathManager() {
}

nav_msgs::Path PathManager::constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z) {
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

bool PathManager::pathCallback(uav_path_manager::GetGeneratedPath::Request &req_path,
                           uav_path_manager::GetGeneratedPath::Response &res_path) {
    res_path.generated_path = path;
    return true;
}

void PathManager::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose = *_ual_pose;
}

void PathManager::ualStateCallback(const uav_abstraction_layer::State &_ual_state) {
    ual_state.state = _ual_state.state;
}

void PathManager::velocityCallback(const geometry_msgs::TwistStamped &_velocity) {
    velocity_ = _velocity;
}

void PathManager::pubMsgs() {
    pub_init_path.publish(init_path);
    pub_generated_path.publish(path);
}

void PathManager::runMission() {
    uav_abstraction_layer::TakeOff take_off;
    uav_abstraction_layer::Land land;
    uav_path_manager::GeneratePath generate_path;
    std_msgs::Int8 generator_mode;
    generator_mode.data = 2;
    generate_path.request.generator_mode = generator_mode;
    generate_path.request.init_path = init_path;
    if (path.poses.size() < 1) {
        srv_generated_path.call(generate_path);
        path = generate_path.response.generated_path;
    }
    Eigen::Vector3f current_p, path0_p, path_end_p;
    current_p = Eigen::Vector3f(ual_pose.pose.position.x, ual_pose.pose.position.y, ual_pose.pose.position.z);
    path0_p = Eigen::Vector3f(path.poses.front().pose.position.x, path.poses.front().pose.position.y, path.poses.front().pose.position.z);
    path_end_p = Eigen::Vector3f(path.poses.back().pose.position.x, path.poses.back().pose.position.y, path.poses.back().pose.position.z);
    switch (ual_state.state) {
        case 2:  // Landed armed
            if (!end_path) {
                take_off.request.height = 5.0;
                take_off.request.blocking = true;
                srv_take_off.call(take_off);
            }
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
                    pub_set_pose.publish(path.poses.back());
                } else {
                    land.request.blocking = true;
                    srv_land.call(land);
                }
            }
            break;
        case 5:  // Landing
            break;
        default:
            break;
    }
}