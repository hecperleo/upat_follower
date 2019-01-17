#include <path_generator_follower/path_manager.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "path_manager_node");

    PathManager manager;

    ros::Rate rate(10);
    while (ros::ok()) {
        manager.runMission();
        manager.pubMsgs();
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}