#include <gtest/gtest.h>
#include <math.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <thread>
#include "uav_path_manager/path_generator.h"

// terminal: catkin build --verbose --catkin-make-args run_tests | sed -n '/\[==========\]/,/\[==========\]/p'

class MyTestSuite : public ::testing::Test {
   public:
    MyTestSuite() {
    }
    ~MyTestSuite() {}
    // TODO: Check if this is correct
    PathGenerator generator;
    std::vector<double> list_pose_x = {5.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0, 15.0, 15.0, 15.0, 20.0, 20.0, 20.0, 20.0};
    std::vector<double> list_pose_y = {5.0, 10.0, 10.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0, 10.0, 10.0};
    std::vector<double> list_pose_z = {10.0, 10.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0};
};

nav_msgs::Path constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z) {
    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses(wps_x.size());
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

nav_msgs::Path csvToPath(std::string file_name) {
    nav_msgs::Path out_path;
    std::stringstream aux_envvar_home(std::getenv("HOME"));
    std::string workspace_name = "/tfm_ws";
    std::string folder_name = aux_envvar_home.str() + workspace_name + "/src/uav_path_manager/data" + file_name;
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
        list_x.pop_back();
        list_y.pop_back();
        list_z.pop_back();
    }

    return constructPath(list_x, list_y, list_z);
}

std::vector<std::vector<double>> csvToVector(std::string file_name) {
    std::vector<std::vector<double>> out_vec;
    std::stringstream aux_envvar_home(std::getenv("HOME"));
    std::string workspace_name = "/tfm_ws";
    std::string folder_name = aux_envvar_home.str() + workspace_name + "/src/uav_path_manager/data" + file_name;
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
        list_x.pop_back();
        list_y.pop_back();
        list_z.pop_back();
    }
    out_vec.push_back(list_x);
    out_vec.push_back(list_y);
    out_vec.push_back(list_z);
    return out_vec;
}

std::vector<double> pathToVector(nav_msgs::Path path, std::string axis) {
    std::vector<double> out_vec(path.poses.size());
    for (int i = 0; i < path.poses.size(); i++) {
        if (axis == "x") {
            out_vec[i] = path.poses.at(i).pose.position.x;
        }
        if (axis == "y") {
            out_vec[i] = path.poses.at(i).pose.position.y;
        }
        if (axis == "z") {
            out_vec[i] = path.poses.at(i).pose.position.z;
        }
    }
    return out_vec;
}

TEST_F(MyTestSuite, interp1) {
    int interp1_final_size = 1000;
    nav_msgs::Path path_interp1 = generator.createPathInterp1(list_pose_x, list_pose_y, list_pose_z, list_pose_x.size(), interp1_final_size);
    EXPECT_EQ(path_interp1.poses.size(), interp1_final_size);
}

TEST_F(MyTestSuite, cubicSpline) {
    nav_msgs::Path path_cubic_spline = generator.createPathCubicSpline(list_pose_x, list_pose_y, list_pose_z, list_pose_x.size());
    nav_msgs::Path compare_path = csvToPath("/generated_path.csv");
    // std::vector<std::vector<double>> compare_path = csvToVector("/generated_path.csv");
    // ASSERT_EQ(path_cubic_spline.poses.size(), compare_path.poses.size());
    std::vector<double> ref_list_x = pathToVector(compare_path, "x");
    std::vector<double> ref_list_y = pathToVector(compare_path, "y");
    std::vector<double> ref_list_z = pathToVector(compare_path, "z");
    // std::vector<double> ref_list_x = compare_path[0];
    // std::vector<double> ref_list_y = compare_path[1];
    // std::vector<double> ref_list_z = compare_path[2];
    std::vector<double> act_list_x = pathToVector(path_cubic_spline, "x");
    std::vector<double> act_list_y = pathToVector(path_cubic_spline, "y");
    std::vector<double> act_list_z = pathToVector(path_cubic_spline, "z");
    ASSERT_EQ(ref_list_x.size(), act_list_x.size());
    ASSERT_EQ(ref_list_y.size(), act_list_y.size());
    ASSERT_EQ(ref_list_z.size(), act_list_z.size());
    float dec = 100.0f;
    for (int i = 0; i < ref_list_x.size(); i++) {
        EXPECT_EQ(roundf(ref_list_x[i]*dec)/dec, roundf(act_list_x[i]*dec)/dec);
        EXPECT_EQ(roundf(ref_list_y[i]*dec)/dec, roundf(act_list_y[i]*dec)/dec);
        EXPECT_EQ(roundf(ref_list_z[i]*dec)/dec, roundf(act_list_z[i]*dec)/dec);
        // EXPECT_EQ(floorf(ref_list_x[i]), floorf(act_list_x[i]));
        // EXPECT_EQ(floorf(ref_list_y[i]), floorf(act_list_y[i]));
        // EXPECT_EQ(floorf(ref_list_z[i]), floorf(act_list_z[i]));
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);

    std::thread t([] {while(ros::ok()) ros::spin(); });

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
