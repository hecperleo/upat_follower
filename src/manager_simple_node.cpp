#include <path_follower/manager_simple.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "manager_simple_node");

    ManagerSimple manager;

    ros::Rate rate(5);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}