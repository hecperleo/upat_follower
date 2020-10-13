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
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, sEXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <uav_abstraction_layer/State.h>
#include <upat_follower/visualization.h>

#include <Eigen/Eigen>
#include <fstream>

geometry_msgs::PoseStamped ual_pose_0_, ual_pose_1_, ual_pose_2_;
uav_abstraction_layer::State ual_state_0_, ual_state_1_, ual_state_2_;

void ualPose0Cb(const geometry_msgs::PoseStamped &_msg) {
    ual_pose_0_ = _msg;
}

void ualPose1Cb(const geometry_msgs::PoseStamped &_msg) {
    ual_pose_1_ = _msg;
}

void ualPose2Cb(const geometry_msgs::PoseStamped &_msg) {
    ual_pose_2_ = _msg;
}

void ualState0Cb(const uav_abstraction_layer::State &_msg) {
    ual_state_0_ = _msg;
}

void ualState1Cb(const uav_abstraction_layer::State &_msg) {
    ual_state_1_ = _msg;
}

void ualState2Cb(const uav_abstraction_layer::State &_msg) {
    ual_state_2_ = _msg;
}

double dist2UAVs(const geometry_msgs::PoseStamped &_p1, const geometry_msgs::PoseStamped &_p2) {
    Eigen::Vector3f p1 = Eigen::Vector3f(_p1.pose.position.x, _p1.pose.position.y, _p1.pose.position.z);
    Eigen::Vector3f p2 = Eigen::Vector3f(_p2.pose.position.x, _p2.pose.position.y, _p2.pose.position.z);

    return (p2 - p1).norm();
}

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "multi_uav_log_node");

    ros::NodeHandle nh;
    std::string ns_prefix;
    ros::param::param<std::string>("~ns_prefix", ns_prefix, "uav_");

    ros::Subscriber sub_state_0_ = nh.subscribe("/" + ns_prefix + "0/ual/state", 0, ualState0Cb);
    ros::Subscriber sub_state_1_ = nh.subscribe("/" + ns_prefix + "1/ual/state", 0, ualState1Cb);
    ros::Subscriber sub_state_2_ = nh.subscribe("/" + ns_prefix + "2/ual/state", 0, ualState2Cb);
    ros::Subscriber sub_pose_0_ = nh.subscribe("/" + ns_prefix + "0/ual/pose", 0, ualPose0Cb);
    ros::Subscriber sub_pose_1_ = nh.subscribe("/" + ns_prefix + "1/ual/pose", 0, ualPose1Cb);
    ros::Subscriber sub_pose_2_ = nh.subscribe("/" + ns_prefix + "2/ual/pose", 0, ualPose2Cb);

    std::string pkg_name_path = ros::package::getPath("upat_follower");
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    std::string folder_data_name = pkg_name_path + "/data/log/" + oss.str();
    if (mkdir((folder_data_name).c_str(), 0777) == -1) ROS_ERROR("Directory creation failed");
    std::ofstream csv_multi_uav;
    csv_multi_uav.open(folder_data_name + "/multi_uav_log.csv");
    csv_multi_uav << std::fixed << std::setprecision(5);

    double actual_time, dist_01, dist_02, dist_12;

    ros::Rate rate(2);
    while (ros::ok()) {
        if (ual_state_0_.state == 4 && ual_state_1_.state == 4 && ual_state_2_.state == 4 ||
            ual_state_0_.state == 6 || ual_state_1_.state == 6 || ual_state_2_.state == 6
            ) {
            static double initial_time = ros::Time::now().toSec();
            actual_time = ros::Time::now().toSec() - initial_time;
            dist_01 = dist2UAVs(ual_pose_0_, ual_pose_1_);
            dist_02 = dist2UAVs(ual_pose_0_, ual_pose_2_);
            dist_12 = dist2UAVs(ual_pose_1_, ual_pose_2_);
            csv_multi_uav << actual_time
                          << ", " << dist_01 << ", " << dist_02 << ", " << dist_12
                          << ", " << ual_pose_0_.pose.position.x << ", " << ual_pose_0_.pose.position.y << ", " << ual_pose_0_.pose.position.z
                          << ", " << ual_pose_1_.pose.position.x << ", " << ual_pose_1_.pose.position.y << ", " << ual_pose_1_.pose.position.z
                          << ", " << ual_pose_2_.pose.position.x << ", " << ual_pose_2_.pose.position.y << ", " << ual_pose_2_.pose.position.z
                          << std::endl;
        }

        if (ros::Time::now().toSec() > 20.0 && ual_state_0_.state == 2 && ual_state_1_.state == 2 && ual_state_2_.state == 2) {
            csv_multi_uav.close();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}