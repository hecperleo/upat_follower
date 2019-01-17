#include <uav_path_manager/path_follower.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "path_follower_node");

    PathFollower path_follower;

    ros::Rate rate(10);
    while (ros::ok()) {
        path_follower.followPath();
        path_follower.pubMsgs();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
