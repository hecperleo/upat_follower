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

#include <upat_follower/ual_communication.h>

namespace upat_follower {

UALCommunication::UALCommunication() : nh_(), pnh_("~") {
    // Parameters
    pnh_.getParam("uav_id", uav_id_);
    pnh_.getParam("ns_prefix", ns_prefix_);
    pnh_.getParam("save_test_data", save_test_);
    pnh_.getParam("trajectory", trajectory_);
    pnh_.getParam("path", init_path_name_);
    pnh_.getParam("pkg_name", pkg_name_);
    pnh_.getParam("reach_tolerance", reach_tolerance_);
    pnh_.getParam("use_class", use_class_);
    pnh_.getParam("generator_mode", generator_mode_);
    pnh_.getParam("cruising_speed", cruising_speed_);
    pnh_.getParam("look_ahead", look_ahead_);
    // ros::param::param<int>("uav_id", uav_id_, 1);
    // ros::param::param<std::string>("ns_prefix", ns_prefix_, "uav_");
    // ros::param::param<bool>("save_test_data", save_test_, "false");
    // ros::param::param<bool>("trajectory", trajectory_, "false");
    // ros::param::param<std::string>("path", init_path_name_);
    // ros::param::param<std::string>("pkg_name", pkg_name_);
    // ros::param::param<double>("reach_tolerance", reach_tolerance_, 0.1);
    // ros::param::param<bool>("use_class", use_class_, true);
    // ros::param::param<int>("generator_mode", generator_mode_, 0);
    ros::param::param<bool>("~debug", debug_, false);
    // Subscriptions
    sub_ual_pose_ = nh_.subscribe("/" + ns_prefix_ + std::to_string(uav_id_) + "/ual/pose", 0, &UALCommunication::ualPoseCallback, this);
    sub_ual_state_ = nh_.subscribe("/" + ns_prefix_ + std::to_string(uav_id_) + "/ual/state", 0, &UALCommunication::ualStateCallback, this);
    sub_ual_velocity_ = nh_.subscribe("/" + ns_prefix_ + std::to_string(uav_id_) + "/ual/velocity", 0, &UALCommunication::ualVelocityCallback, this);
    sub_velocity_ = nh_.subscribe("/" + ns_prefix_ + std::to_string(uav_id_) + "/upat_follower/follower/output_vel", 0, &UALCommunication::velocityCallback, this);
    // Publishers
    pub_set_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + ns_prefix_ + std::to_string(uav_id_) + "/ual/set_pose", 1000);
    pub_set_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>("/" + ns_prefix_ + std::to_string(uav_id_) + "/ual/set_velocity", 1000);
    // Services
    client_go_to_waypoint_ = nh_.serviceClient<uav_abstraction_layer::GoToWaypoint>("/" + ns_prefix_ + std::to_string(uav_id_) + "/ual/go_to_waypoint");
    client_take_off_ = nh_.serviceClient<uav_abstraction_layer::TakeOff>("/" + ns_prefix_ + std::to_string(uav_id_) + "/ual/take_off");
    client_land_ = nh_.serviceClient<uav_abstraction_layer::Land>("/" + ns_prefix_ + std::to_string(uav_id_) + "/ual/land");
    client_prepare_path_ = nh_.serviceClient<upat_follower::PreparePath>("/" + ns_prefix_ + std::to_string(uav_id_) + "/upat_follower/follower/prepare_path");
    client_prepare_trajectory_ = nh_.serviceClient<upat_follower::PrepareTrajectory>("/" + ns_prefix_ + std::to_string(uav_id_) + "/upat_follower/follower/prepare_trajectory");
    client_visualize_ = nh_.serviceClient<upat_follower::Visualize>("/" + ns_prefix_ + std::to_string(uav_id_) + "/upat_follower/visualization/visualize");
    // Initialize path
    init_path_ = csvToPath("/config/" + init_path_name_ + ".csv");
    times_ = csvToVector("/config/" + init_path_name_ + "_t.csv");
    // Save data
    if (save_test_) {
        std::string pkg_name_path = ros::package::getPath(pkg_name_);
        folder_data_name_ = pkg_name_path + "/tests/splines";
    }
}

UALCommunication::~UALCommunication() {
}

nav_msgs::Path UALCommunication::constructPath(std::vector<double> _wps_x, std::vector<double> _wps_y, std::vector<double> _wps_z, std::string frame_id) {
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

nav_msgs::Path UALCommunication::csvToPath(std::string _file_name) {
    nav_msgs::Path out_path;
    std::string pkg_name_path = ros::package::getPath(pkg_name_);
    std::string folder_name = pkg_name_path + _file_name;
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

    return constructPath(list_x, list_y, list_z, ns_prefix_ + std::to_string(uav_id_) + "/odom");
}

std::vector<double> UALCommunication::csvToVector(std::string _file_name) {
    std::vector<double> out_vector;
    std::string pkg_name_path = ros::package::getPath(pkg_name_);
    std::string folder_name = pkg_name_path + _file_name;
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

void UALCommunication::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose_ = *_ual_pose;
}

void UALCommunication::ualStateCallback(const uav_abstraction_layer::State &_ual_state) {
    ual_state_.state = _ual_state.state;
}

void UALCommunication::ualVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &_ual_velocity) {
    ual_vel_ = *_ual_velocity;
}

void UALCommunication::velocityCallback(const geometry_msgs::TwistStamped &_velocity) {
    velocity_ = _velocity;
}

void UALCommunication::saveDataForTesting() {
    ROS_WARN("Saving Data For Testing ...");
    static upat_follower::Follower follower_save_tests(uav_id_);
    std::ofstream csv_smooth_spline, csv_cubic_spline, csv_interp1, csv_init, csv_trajectory_m0, csv_trajectory_m1, csv_trajectory_m2, csv_trajectory_t;
    csv_init.open(folder_data_name_ + "/init.csv");
    csv_init << std::fixed << std::setprecision(5);
    for (int i = 0; i < init_path_.poses.size(); i++) {
        csv_init << init_path_.poses.at(i).pose.position.x << ", " << init_path_.poses.at(i).pose.position.y << ", " << init_path_.poses.at(i).pose.position.z << std::endl;
    }
    csv_init.close();
    csv_trajectory_t.open(folder_data_name_ + "/trajectory_t.csv");
    csv_trajectory_t << std::fixed << std::setprecision(5);
    std::cout << "T: ";
    for (int i = 0; i < times_.size(); i++) {
        std::cout << times_.at(i) << " ";
        csv_trajectory_t << times_.at(i) << std::endl;
    }
    std::cout << std::endl;
    csv_trajectory_t.close();
    target_path_ = follower_save_tests.prepareTrajectory(init_path_, times_, 0);
    csv_trajectory_m0.open(folder_data_name_ + "/trajectory_m0.csv");
    csv_trajectory_m0 << std::fixed << std::setprecision(5);
    for (int i = 0; i < target_path_.poses.size(); i++) {
        csv_trajectory_m0 << target_path_.poses.at(i).pose.position.x << ", " << target_path_.poses.at(i).pose.position.y << ", " << target_path_.poses.at(i).pose.position.z << std::endl;
    }
    // csv_trajectory_m0.close();
    // target_path_ = follower_save_tests.prepareTrajectory(init_path_, times_, 1);
    // csv_trajectory_m1.open(folder_data_name_ + "/trajectory_m1.csv");
    // csv_trajectory_m1 << std::fixed << std::setprecision(5);
    // for (int i = 0; i < target_path_.poses.size(); i++) {
    //     csv_trajectory_m1 << target_path_.poses.at(i).pose.position.x << ", " << target_path_.poses.at(i).pose.position.y << ", " << target_path_.poses.at(i).pose.position.z << std::endl;
    // }
    // csv_trajectory_m1.close();
    // target_path_ = follower_save_tests.prepareTrajectory(init_path_, times_, 2);
    // csv_trajectory_m2.open(folder_data_name_ + "/trajectory_m2.csv");
    // csv_trajectory_m2 << std::fixed << std::setprecision(5);
    // for (int i = 0; i < target_path_.poses.size(); i++) {
    //     csv_trajectory_m2 << target_path_.poses.at(i).pose.position.x << ", " << target_path_.poses.at(i).pose.position.y << ", " << target_path_.poses.at(i).pose.position.z << std::endl;
    // }
    // csv_trajectory_m2.close();
    target_path_ = follower_save_tests.preparePath(init_path_, 0);
    csv_interp1.open(folder_data_name_ + "/interp1.csv");
    csv_interp1 << std::fixed << std::setprecision(5);
    for (int i = 0; i < target_path_.poses.size(); i++) {
        csv_interp1 << target_path_.poses.at(i).pose.position.x << ", " << target_path_.poses.at(i).pose.position.y << ", " << target_path_.poses.at(i).pose.position.z << std::endl;
    }
    csv_interp1.close();
    // target_path_ = follower_save_tests.preparePath(init_path_, 1);
    // csv_smooth_spline.open(folder_data_name_ + "/smooth_spline.csv");
    // csv_smooth_spline << std::fixed << std::setprecision(5);
    // for (int i = 0; i < target_path_.poses.size(); i++) {
    //     csv_smooth_spline << target_path_.poses.at(i).pose.position.x << ", " << target_path_.poses.at(i).pose.position.y << ", " << target_path_.poses.at(i).pose.position.z << std::endl;
    // }
    // csv_smooth_spline.close();
    // target_path_ = follower_save_tests.preparePath(init_path_, 2);
    // csv_cubic_spline.open(folder_data_name_ + "/cubic_spline.csv");
    // csv_cubic_spline << std::fixed << std::setprecision(5);
    // for (int i = 0; i < target_path_.poses.size(); i++) {
    //     csv_cubic_spline << target_path_.poses.at(i).pose.position.x << ", " << target_path_.poses.at(i).pose.position.y << ", " << target_path_.poses.at(i).pose.position.z << std::endl;
    // }
    // csv_cubic_spline.close();
}

void UALCommunication::callVisualization() {
    upat_follower::Visualize visualize;
    visualize.request.init_path = init_path_;
    visualize.request.generated_path = target_path_;
    visualize.request.current_path = current_path_;
    visualize.request.current_vel = ual_vel_;
    visualize.request.desired_vel = velocity_;
    client_visualize_.call(visualize);
}

void UALCommunication::switchState(state_t new_state) {
    state_ = new_state;
    switch (state_) {
        case hover_:
            ROS_INFO("[%d][UPAT] State switched to hover.", uav_id_);
            break;
        case go_to_start_:
            ROS_INFO("[%d][UPAT] State switched to go_to_start.", uav_id_);
            break;
        case go_to_end_:
            ROS_INFO("[%d][UPAT] State switched to go_to_end.", uav_id_);
            break;
        case execute_path_:
            start_count_time_ = ros::Time::now().toSec();
            ROS_INFO("[%d][UPAT] State switched to execute_path.", uav_id_);
            break;
        case hover_emergency_:
            ROS_WARN("[%d][UPAT] State switched to hover_emergency.", uav_id_);
            break;
        default:
            break;
    }
}

void UALCommunication::runMission() {
    static upat_follower::Follower follower_(uav_id_, debug_);

    uav_abstraction_layer::TakeOff take_off;
    uav_abstraction_layer::Land land;
    uav_abstraction_layer::GoToWaypoint go_to_waypoint_back;
    uav_abstraction_layer::GoToWaypoint go_to_waypoint_front;
    uav_abstraction_layer::GoToWaypoint go_to_waypoint_hover;
    upat_follower::PreparePath prepare_path;
    upat_follower::PrepareTrajectory prepare_trajectory;
    if (flag_redo_) {
        if (save_test_) saveDataForTesting();
        if (trajectory_) {
            for (int i = 0; i < times_.size(); i++) {
                std_msgs::Float32 v_percentage;
                v_percentage.data = times_[i];
                prepare_trajectory.request.times.push_back(v_percentage);
            }
            prepare_trajectory.request.init_path = init_path_;
            prepare_trajectory.request.generator_mode.data = generator_mode_;
            if (!use_class_) {
                client_prepare_trajectory_.call(prepare_trajectory);
                target_path_ = prepare_trajectory.response.generated_path;
            }
            if (use_class_) target_path_ = follower_.prepareTrajectory(init_path_, times_, generator_mode_, look_ahead_);
        } else {
            prepare_path.request.init_path = init_path_;
            prepare_path.request.generator_mode.data = generator_mode_;
            prepare_path.request.look_ahead.data = look_ahead_;
            prepare_path.request.cruising_speed.data = cruising_speed_;
            if (!use_class_) {
                client_prepare_path_.call(prepare_path);
                target_path_ = prepare_path.response.generated_path;
            }
            if (use_class_) target_path_ = follower_.preparePath(init_path_, generator_mode_, look_ahead_, cruising_speed_);
        }
        flag_redo_ = false;
    } else if (flag_update_) {
        follower_.updatePath(target_path_);
        flag_update_ = false;
    }

    go_to_waypoint_back.request.waypoint.header = target_path_.poses.back().header;
    go_to_waypoint_back.request.waypoint.pose = target_path_.poses.back().pose;
    go_to_waypoint_back.request.blocking = true;
    go_to_waypoint_front.request.waypoint.header = target_path_.poses.front().header;
    go_to_waypoint_front.request.waypoint.pose = target_path_.poses.front().pose;
    go_to_waypoint_front.request.blocking = true;
    go_to_waypoint_hover.request.waypoint.header = hover_emergency_pose_.header;
    go_to_waypoint_hover.request.waypoint.pose = hover_emergency_pose_.pose;
    Eigen::Vector3f current_p, path0_p, path_end_p;
    current_p = Eigen::Vector3f(ual_pose_.pose.position.x, ual_pose_.pose.position.y, ual_pose_.pose.position.z);
    path0_p = Eigen::Vector3f(target_path_.poses.front().pose.position.x, target_path_.poses.front().pose.position.y, target_path_.poses.front().pose.position.z);
    path_end_p = Eigen::Vector3f(target_path_.poses.back().pose.position.x, target_path_.poses.back().pose.position.y, target_path_.poses.back().pose.position.z);

    switch (ual_state_.state) {
        case 2:  // Landed armed
            break;
        case 3:  // Taking of
            break;
        case 4:  // Flying auto
            switch (state_) {
                case hover_:
                    break;
                case go_to_start_:
                    client_go_to_waypoint_.call(go_to_waypoint_front);
                    switchState(execute_path_);
                    break;
                case execute_path_:
                    if (reach_tolerance_ > (current_p - path_end_p).norm()) {
                        switchState(go_to_end_);
                    } else {
                        if (use_class_) {
                            follower_.updatePose(ual_pose_);
                            static double start_time = ros::Time::now().toSec();
                            if (trajectory_) follower_.actual_time_ = ros::Time::now().toSec() - start_time;
                            velocity_ = follower_.getVelocity();
                            // std::cout << "|V| = " << sqrt(velocity_.twist.linear.x * velocity_.twist.linear.x + velocity_.twist.linear.y * velocity_.twist.linear.y + velocity_.twist.linear.z * velocity_.twist.linear.z) << std::endl;
                            if (debug_) follower_.pubMsgs();
                            position_on_path_ = follower_.position_on_path_;
                        }
                        pub_set_velocity_.publish(velocity_);
                        current_path_.header.frame_id = ual_pose_.header.frame_id;
                        current_path_.poses.push_back(ual_pose_);
                    }
                    break;
                case go_to_end_:
                    ROS_INFO_COND(trajectory_, "[%d][UPAT] Path executed in %.2f seconds.", uav_id_, ros::Time::now().toSec() - start_count_time_);
                    // client_go_to_waypoint_.call(go_to_waypoint_back);  // Comment to save data on experiments
                    client_land_.call(land);  // Uncomment to save data on experiments
                    switchState(hover_);

                    break;
                case hover_emergency_:
                    pub_set_pose_.publish(hover_emergency_pose_);
                    break;
            }
            break;
        case 5:  // Landing
            break;
    }
}

}  // namespace upat_follower