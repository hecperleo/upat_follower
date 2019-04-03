#include <gtest/gtest.h>
#include <math.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <uav_path_manager/generator.h>
#include <fstream>
#include <string>
#include <thread>

// terminal: catkin build --verbose --catkin-make-args run_tests | sed -n '/\[==========\]/,/\[==========\]/p'

class MyTestSuite : public ::testing::Test {
   public:
    MyTestSuite() {
    }
    ~MyTestSuite() {}
    float tolerance = 0.0001;
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
    std::string folder_name = pkg_name_path + "/tests/data" + file_name;
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

TEST_F(MyTestSuite, interp1) {
    uav_path_manager::Generator generator_(2.0, 3.0, 1.0);
    nav_msgs::Path init_path = csvToPath("/init.csv");
    nav_msgs::Path ref_path = csvToPath("/interp1.csv");
    nav_msgs::Path act_path = generator_.generatePath(init_path, 0);
    ASSERT_EQ(ref_path.poses.size(), act_path.poses.size());
    for (int i = 0; i < ref_path.poses.size(); i++) {
        EXPECT_NEAR(ref_path.poses.at(i).pose.position.x, act_path.poses.at(i).pose.position.x, tolerance);
        EXPECT_NEAR(ref_path.poses.at(i).pose.position.y, act_path.poses.at(i).pose.position.y, tolerance);
        EXPECT_NEAR(ref_path.poses.at(i).pose.position.z, act_path.poses.at(i).pose.position.z, tolerance);
    }
}

TEST_F(MyTestSuite, cubicSplineLoyal) {
    uav_path_manager::Generator generator_(2.0, 3.0, 1.0);
    nav_msgs::Path init_path = csvToPath("/init.csv");
    nav_msgs::Path ref_path = csvToPath("/cubic_spline_loyal.csv");
    nav_msgs::Path act_path = generator_.generatePath(init_path, 1);
    ASSERT_EQ(ref_path.poses.size(), act_path.poses.size());
    for (int i = 0; i < ref_path.poses.size(); i++) {
        EXPECT_NEAR(ref_path.poses.at(i).pose.position.x, act_path.poses.at(i).pose.position.x, tolerance);
        EXPECT_NEAR(ref_path.poses.at(i).pose.position.y, act_path.poses.at(i).pose.position.y, tolerance);
        EXPECT_NEAR(ref_path.poses.at(i).pose.position.z, act_path.poses.at(i).pose.position.z, tolerance);
    }
}

TEST_F(MyTestSuite, cubicSpline) {
    uav_path_manager::Generator generator_(2.0, 3.0, 1.0);
    nav_msgs::Path init_path = csvToPath("/init.csv");
    nav_msgs::Path ref_path = csvToPath("/cubic_spline.csv");
    nav_msgs::Path act_path = generator_.generatePath(init_path, 2);
    ASSERT_EQ(ref_path.poses.size(), act_path.poses.size());
    for (int i = 0; i < ref_path.poses.size(); i++) {
        EXPECT_NEAR(ref_path.poses.at(i).pose.position.x, act_path.poses.at(i).pose.position.x, tolerance);
        EXPECT_NEAR(ref_path.poses.at(i).pose.position.y, act_path.poses.at(i).pose.position.y, tolerance);
        EXPECT_NEAR(ref_path.poses.at(i).pose.position.z, act_path.poses.at(i).pose.position.z, tolerance);
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
