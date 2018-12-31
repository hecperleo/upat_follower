#include "path_follower/manager.h"
#include <ros/ros.h>

#include <gtest/gtest.h>
#include <thread>

// terminal: catkin build --verbose --catkin-make-args run_tests | sed -n '/\[==========\]/,/\[==========\]/p'

class MyTestSuite : public ::testing::Test {
   public:
    MyTestSuite() {
    }
    ~MyTestSuite() {}
};

TEST_F(MyTestSuite, modValue) {
    Manager mg;
    float x1, y1, z1, x2, y2, z2;
    x1 = y1 = z1 = 1;
    x2 = y2 = z2 = 10;
    float dist = mg.funcMod(x1, x2, y1, y2, z1, z2);
    EXPECT_EQ(dist, (float)sqrt(243));
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
