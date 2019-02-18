#include <uav_path_manager/path_follower.h>
#include <ros/ros.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "path_follower_node");

    PathFollower path_follower;
    int pub_rate_;
    ros::param::param<int>("~pub_rate",pub_rate_,30);
    ros::Rate rate(pub_rate_);
    while (ros::ok()) {
        path_follower.followPath();
        path_follower.pubMsgs();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
