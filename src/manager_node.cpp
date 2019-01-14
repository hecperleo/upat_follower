#include <path_generator_follower/manager.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "manager_node");

    Manager manager;

    while (ros::ok()) {
        sleep(0.5);
    }

    return 0;
}