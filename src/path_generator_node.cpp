#include <uav_path_manager/path_generator.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "path_generator_node");

    PathGenerator generator;

    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}