#include <uav_path_manager/path_generator.h>
#include <ros/ros.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "path_generator_node");

    PathGenerator generator;
    int pub_rate_;
    ros::param::param<int>("~pub_rate",pub_rate_,30);
    ros::Rate rate(pub_rate_);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}