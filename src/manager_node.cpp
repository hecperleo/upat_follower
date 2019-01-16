#include <path_generator_follower/manager.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "manager_node");

    Manager manager;

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}