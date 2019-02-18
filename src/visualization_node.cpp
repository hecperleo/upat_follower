#include <uav_path_manager/visualization.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "visualization_node");

    Visualization visual;

    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}