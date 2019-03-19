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

#include <uav_path_manager/path_manager.h>

PathManager::PathManager() : nh_(), pnh_("~") {
    // Parameters
    pnh_.getParam("uav_id", uav_id_);
    pnh_.getParam("save_csv", save_csv_);
    pnh_.getParam("trajectory", trajectory_);
    pnh_.getParam("path", init_path_name_);
    pnh_.getParam("reach_tolerance", reach_tolerance_);
    // Subscriptions
    sub_pose_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/pose", 0, &PathManager::ualPoseCallback, this);
    sub_state_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/state", 0, &PathManager::ualStateCallback, this);
    sub_velocity_ = nh_.subscribe("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/output_vel", 0, &PathManager::velocityCallback, this);
    // Publishers
    pub_set_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav_" + std::to_string(uav_id_) + "/ual/set_pose", 1000);
    pub_set_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>("/uav_" + std::to_string(uav_id_) + "/ual/set_velocity", 1000);
    // Services
    client_take_off_ = nh_.serviceClient<uav_abstraction_layer::TakeOff>("/uav_" + std::to_string(uav_id_) + "/ual/take_off");
    client_land_ = nh_.serviceClient<uav_abstraction_layer::Land>("/uav_" + std::to_string(uav_id_) + "/ual/land");
    client_generate_path_ = nh_.serviceClient<uav_path_manager::GeneratePath>("/uav_path_manager/generator/generate_path");
    client_follow_path_ = nh_.serviceClient<uav_path_manager::FollowPath>("/uav_path_manager/follower/uav_" + std::to_string(uav_id_) + "/follow_path");
    client_visualize_ = nh_.serviceClient<uav_path_manager::Visualize>("/uav_path_manager/visualization/uav_" + std::to_string(uav_id_) + "/visualize");
    // Flags
    on_path_ = false;
    end_path_ = false;
    // Initialize path
    init_path_ = csvToPath("/" + init_path_name_ + ".csv");
    max_vel_percentage_ = csvToVector("/velocities.csv");
    // Save data
    if (save_csv_) {
        std::string pkg_name_path = ros::package::getPath("uav_path_manager");
        folder_data_name_ = pkg_name_path + "/tests/data";
    }
}

PathManager::~PathManager() {
}

nav_msgs::Path PathManager::constructPath(std::vector<double> _wps_x, std::vector<double> _wps_y, std::vector<double> _wps_z, std::string frame_id) {
    nav_msgs::Path out_path;
    std::vector<geometry_msgs::PoseStamped> poses(_wps_x.size());
    out_path.header.frame_id = frame_id;
    for (int i = 0; i < _wps_x.size(); i++) {
        poses.at(i).pose.position.x = _wps_x[i];
        poses.at(i).pose.position.y = _wps_y[i];
        poses.at(i).pose.position.z = _wps_z[i];
        poses.at(i).pose.orientation.x = 0;
        poses.at(i).pose.orientation.y = 0;
        poses.at(i).pose.orientation.z = 0;
        poses.at(i).pose.orientation.w = 1;
    }
    out_path.poses = poses;
    return out_path;
}

nav_msgs::Path PathManager::csvToPath(std::string _file_name) {
    nav_msgs::Path out_path;
    std::string pkg_name_path = ros::package::getPath("uav_path_manager");
    std::string folder_name = pkg_name_path + "/data" + _file_name;
    std::fstream read_csv;
    read_csv.open(folder_name);
    std::vector<double> list_x, list_y, list_z;
    if (read_csv.is_open()) {
        while (read_csv.good()) {
            std::string x, y, z;
            double dx, dy, dz;
            getline(read_csv, x, ',');
            getline(read_csv, y, ',');
            getline(read_csv, z, '\n');
            std::stringstream sx(x);
            std::stringstream sy(y);
            std::stringstream sz(z);
            sx >> dx;
            sy >> dy;
            sz >> dz;
            list_x.push_back(dx);
            list_y.push_back(dy);
            list_z.push_back(dz);
        }
    }

    return constructPath(list_x, list_y, list_z, "uav_" + std::to_string(uav_id_) + "_home");
}

std::vector<double> PathManager::csvToVector(std::string _file_name) {
    std::vector<double> out_vector;
    std::string pkg_name_path = ros::package::getPath("uav_path_manager");
    std::string folder_name = pkg_name_path + "/data" + _file_name;
    std::fstream read_csv;
    read_csv.open(folder_name);
    if (read_csv.is_open()) {
        while (read_csv.good()) {
            std::string x;
            double dx;
            getline(read_csv, x, '\n');
            std::stringstream sx(x);
            sx >> dx;
            out_vector.push_back(dx);
        }
    }

    return out_vector;
}

void PathManager::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose_ = *_ual_pose;
}

void PathManager::ualStateCallback(const uav_abstraction_layer::State &_ual_state) {
    ual_state_.state = _ual_state.state;
}

void PathManager::velocityCallback(const geometry_msgs::TwistStamped &_velocity) {
    velocity_ = _velocity;
}

void PathManager::saveDataForTesting() {
    uav_path_manager::GeneratePath generate_path;
    std_msgs::Int8 generator_mode;
    std::ofstream csv_cubic_loyal, csv_cubic, csv_interp1, csv_init;
    csv_init.open(folder_data_name_ + "/init.csv");
    csv_init << std::fixed << std::setprecision(5);
    for (int i = 0; i < init_path_.poses.size(); i++) {
        csv_init << init_path_.poses.at(i).pose.position.x << ", " << init_path_.poses.at(i).pose.position.y << ", " << init_path_.poses.at(i).pose.position.z << std::endl;
    }
    csv_init.close();
    generator_mode.data = 1;
    generate_path.request.generator_mode = generator_mode;
    generate_path.request.init_path = init_path_;
    client_generate_path_.call(generate_path);
    path = generate_path.response.generated_path;
    csv_interp1.open(folder_data_name_ + "/interp1.csv");
    csv_interp1 << std::fixed << std::setprecision(5);
    for (int i = 0; i < path.poses.size(); i++) {
        csv_interp1 << path.poses.at(i).pose.position.x << ", " << path.poses.at(i).pose.position.y << ", " << path.poses.at(i).pose.position.z << std::endl;
    }
    csv_interp1.close();
    generator_mode.data = 2;
    generate_path.request.generator_mode = generator_mode;
    client_generate_path_.call(generate_path);
    path = generate_path.response.generated_path;
    csv_cubic_loyal.open(folder_data_name_ + "/cubic_spline_loyal.csv");
    csv_cubic_loyal << std::fixed << std::setprecision(5);
    for (int i = 0; i < path.poses.size(); i++) {
        csv_cubic_loyal << path.poses.at(i).pose.position.x << ", " << path.poses.at(i).pose.position.y << ", " << path.poses.at(i).pose.position.z << std::endl;
    }
    csv_cubic_loyal.close();
    generator_mode.data = 3;
    generate_path.request.generator_mode = generator_mode;
    client_generate_path_.call(generate_path);
    path = generate_path.response.generated_path;
    csv_cubic.open(folder_data_name_ + "/cubic_spline.csv");
    csv_cubic << std::fixed << std::setprecision(5);
    for (int i = 0; i < path.poses.size(); i++) {
        csv_cubic << path.poses.at(i).pose.position.x << ", " << path.poses.at(i).pose.position.y << ", " << path.poses.at(i).pose.position.z << std::endl;
    }
    csv_cubic.close();
}

void PathManager::callVisualization() {
    uav_path_manager::Visualize visualize;
    visualize.request.init_path = init_path_;
    visualize.request.generated_path = path;
    visualize.request.current_path = current_path_;
    client_visualize_.call(visualize);
}

void PathManager::runMission() {
    uav_abstraction_layer::TakeOff take_off;
    uav_abstraction_layer::Land land;
    uav_path_manager::GeneratePath generate_path;
    uav_path_manager::FollowPath follow_path;
    std_msgs::Float32 cruising_speed, look_ahead;
    std_msgs::Int8 generator_mode, follower_mode;
    if (path.poses.size() < 1) {
        if (save_csv_) saveDataForTesting();
        if (trajectory_) {
            generate_path.request.init_path = init_path_;
            for (int i = 0; i < max_vel_percentage_.size(); i++) {
                std_msgs::Float32 v_percentage;
                v_percentage.data = max_vel_percentage_[i];
                generate_path.request.max_vel_percentage.push_back(v_percentage);
            }
            generator_mode.data = 4;
            generate_path.request.generator_mode = generator_mode;
            client_generate_path_.call(generate_path);
            path = generate_path.response.generated_path;
            follower_mode.data = 2;
            follow_path.request.follower_mode = follower_mode;
            follow_path.request.generated_path = path;
            follow_path.request.generated_path_vel_percentage = generate_path.response.generated_path_vel_percentage;
            follow_path.request.generated_max_vel_percentage = generate_path.response.generated_max_vel_percentage;
            follow_path.request.max_velocity = generate_path.response.max_velocity;
            client_follow_path_.call(follow_path);
        } else {
            generator_mode.data = 3;
            generate_path.request.generator_mode = generator_mode;
            generate_path.request.init_path = init_path_;
            client_generate_path_.call(generate_path);
            path = generate_path.response.generated_path;
            follow_path.request.generated_path = path;
            cruising_speed.data = 1.0;
            look_ahead.data = 1.2;
            follower_mode.data = 1;
            follow_path.request.cruising_speed = cruising_speed;
            follow_path.request.look_ahead = look_ahead;
            follow_path.request.follower_mode = follower_mode;
            client_follow_path_.call(follow_path);
        }
    }

    Eigen::Vector3f current_p, path0_p, path_end_p;
    current_p = Eigen::Vector3f(ual_pose_.pose.position.x, ual_pose_.pose.position.y, ual_pose_.pose.position.z);
    path0_p = Eigen::Vector3f(path.poses.front().pose.position.x, path.poses.front().pose.position.y, path.poses.front().pose.position.z);
    path_end_p = Eigen::Vector3f(path.poses.back().pose.position.x, path.poses.back().pose.position.y, path.poses.back().pose.position.z);
    switch (ual_state_.state) {
        case 2:  // Landed armed
            if (!end_path_) {
                take_off.request.height = 5.0;
                take_off.request.blocking = true;
                client_take_off_.call(take_off);
            }
            break;
        case 3:  // Taking of
            break;
        case 4:  // Flying auto
            if (!end_path_) {
                if (!on_path_) {
                    if ((current_p - path0_p).norm() > reach_tolerance_ * 2) {
                        pub_set_pose_.publish(path.poses.at(0));
                    } else if (reach_tolerance_ > (current_p - path0_p).norm()) {
                        pub_set_pose_.publish(path.poses.front());
                        on_path_ = true;
                    }
                } else {
                    if (reach_tolerance_ * 2 > (current_p - path_end_p).norm()) {
                        pub_set_pose_.publish(path.poses.back());
                        on_path_ = false;
                        end_path_ = true;
                    } else {
                        pub_set_velocity_.publish(velocity_);
                        current_path_.header.frame_id = ual_pose_.header.frame_id;
                        current_path_.poses.push_back(ual_pose_);
                    }
                }
            } else {
                if (reach_tolerance_ * 2 > (current_p - path_end_p).norm() && (current_p - path_end_p).norm() > reach_tolerance_) {
                    pub_set_pose_.publish(path.poses.back());
                } else {
                    land.request.blocking = true;
                    client_land_.call(land);
                }
            }
            break;
        case 5:  // Landing
            break;
    }
}