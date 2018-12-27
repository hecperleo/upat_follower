#include <path_follower/initiator.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "initiator_node");

    Initiator initiator;

    while (ros::ok()) {
        sleep(1);
    }

    return 0;
}