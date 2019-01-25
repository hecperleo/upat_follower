#include <gtest/gtest.h>
#include <math.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <thread>
#include "uav_path_manager/path_generator.h"
#include <ros/package.h>

// terminal: catkin build --verbose --catkin-make-args run_tests | sed -n '/\[==========\]/,/\[==========\]/p'

class MyTestSuite : public ::testing::Test {
   public:
    MyTestSuite() {
    }
    ~MyTestSuite() {}
    PathGenerator generator;
    float dec = 1000.0f;
    float tolerance = 10 / dec;
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
    std::string pkg_name_path = ros::package::getPath("uav_path_manager");
    std::string folder_name =  pkg_name_path + "/data" + file_name;
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
    int interp1_final_size = 10000;
    nav_msgs::Path init_path = csvToPath("/init.csv");
    nav_msgs::Path ref_path = csvToPath("/interp1.csv");
    std::vector<double> list_pose_x = pathToVector(init_path, "x");
    std::vector<double> list_pose_y = pathToVector(init_path, "y");
    std::vector<double> list_pose_z = pathToVector(init_path, "z");
    nav_msgs::Path act_path = generator.createPathInterp1(list_pose_x, list_pose_y, list_pose_z, list_pose_x.size(), interp1_final_size);
    ASSERT_EQ(ref_path.poses.size(), act_path.poses.size());
    for (int i = 0; i < ref_path.poses.size(); i++) {
        EXPECT_EQ(roundf(ref_path.poses.at(i).pose.position.x * dec) / dec, roundf(act_path.poses.at(i).pose.position.x * dec) / dec);
        EXPECT_EQ(roundf(ref_path.poses.at(i).pose.position.y * dec) / dec, roundf(act_path.poses.at(i).pose.position.y * dec) / dec);
        EXPECT_EQ(roundf(ref_path.poses.at(i).pose.position.z * dec) / dec, roundf(act_path.poses.at(i).pose.position.z * dec) / dec);
    }
}

TEST_F(MyTestSuite, cubicSpline) {
    generator.mode = generator.mode_cubic_spline;
    nav_msgs::Path init_path = csvToPath("/init.csv");
    nav_msgs::Path ref_path = csvToPath("/cubic_spline.csv");
    std::vector<double> list_pose_x = pathToVector(init_path, "x");
    std::vector<double> list_pose_y = pathToVector(init_path, "y");
    std::vector<double> list_pose_z = pathToVector(init_path, "z");
    nav_msgs::Path act_path = generator.createPathCubicSpline(list_pose_x, list_pose_y, list_pose_z, list_pose_x.size());
    ASSERT_EQ(ref_path.poses.size(), act_path.poses.size());
    for (int i = 0; i < ref_path.poses.size(); i++) {
        EXPECT_NEAR(roundf(ref_path.poses.at(i).pose.position.x * dec) / dec, roundf(act_path.poses.at(i).pose.position.x * dec) / dec, tolerance);
        EXPECT_NEAR(roundf(ref_path.poses.at(i).pose.position.y * dec) / dec, roundf(act_path.poses.at(i).pose.position.y * dec) / dec, tolerance);
        EXPECT_NEAR(roundf(ref_path.poses.at(i).pose.position.z * dec) / dec, roundf(act_path.poses.at(i).pose.position.z * dec) / dec, tolerance);
    }
}

TEST_F(MyTestSuite, cubicSplineLoyal) {
    generator.mode = generator.mode_cubic_spline_loyal;
    nav_msgs::Path init_path = csvToPath("/init.csv");
    nav_msgs::Path ref_path = csvToPath("/cubic_spline_loyal.csv");
    std::vector<double> list_pose_x = pathToVector(init_path, "x");
    std::vector<double> list_pose_y = pathToVector(init_path, "y");
    std::vector<double> list_pose_z = pathToVector(init_path, "z");
    nav_msgs::Path act_path = generator.createPathCubicSpline(list_pose_x, list_pose_y, list_pose_z, list_pose_x.size());
    ASSERT_EQ(ref_path.poses.size(), act_path.poses.size());
    for (int i = 0; i < ref_path.poses.size(); i++) {
        EXPECT_NEAR(roundf(ref_path.poses.at(i).pose.position.x * dec) / dec, roundf(act_path.poses.at(i).pose.position.x * dec) / dec, tolerance);
        EXPECT_NEAR(roundf(ref_path.poses.at(i).pose.position.y * dec) / dec, roundf(act_path.poses.at(i).pose.position.y * dec) / dec, tolerance);
        EXPECT_NEAR(roundf(ref_path.poses.at(i).pose.position.z * dec) / dec, roundf(act_path.poses.at(i).pose.position.z * dec) / dec, tolerance);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tests_node");
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);

    std::thread t([] {while(ros::ok()) ros::spin(); });

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
