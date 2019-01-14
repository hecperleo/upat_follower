#include <path_generator_follower/path_follower.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "path_follower_node");

    PathFollower path_follower;

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        path_follower.pubMsgs();
        rate.sleep();
    }

    return 0;
}
