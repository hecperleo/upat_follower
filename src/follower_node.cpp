#include <path_follower/follower.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "follower_node");

    Follower follower;

    while (ros::ok()) {
        sleep(0.1);
    }

    return 0;
}
