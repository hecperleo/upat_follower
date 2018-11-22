#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include <Eigen/Eigen>

nav_msgs::Path uav_path_actual, uav_path_intendeed;
bool next_wp = false;
int cont_init_d_to_target = 0;
float init_d_to_target = 0;
int cont_smooth_vel = 1;
int max_velocity_portions = 50;

geometry_msgs::TwistStamped calculateSmoothVelocity(geometry_msgs::PoseStamped pose, geometry_msgs::Point target) {
    double cruising_speed = 1.0;
    geometry_msgs::TwistStamped output_vel;
    geometry_msgs::Point current = pose.pose.position;

    Eigen::Vector3f x0 = Eigen::Vector3f(current.x, current.y, current.z);
    Eigen::Vector3f x2 = Eigen::Vector3f(target.x, target.y, target.z);
    // Eigen::Vector3f x1 = Eigen::Vector3f(initial.x, initial.y, initial.z); for pure pursuit ??

    double d_to_target = (x2 - x0).norm();
    Eigen::Vector3f unit_vec = (x2 - x0) / d_to_target;
    double velocity_portion = cruising_speed / max_velocity_portions;

    if (cont_init_d_to_target == 0) {
        init_d_to_target = d_to_target;
        cont_init_d_to_target++;
    }

    output_vel.twist.linear.x = unit_vec(0) * velocity_portion * cont_smooth_vel;
    output_vel.twist.linear.y = unit_vec(1) * velocity_portion * cont_smooth_vel;
    output_vel.twist.linear.z = unit_vec(2) * velocity_portion * cont_smooth_vel;

    if (d_to_target >= 3) {
        if (cont_smooth_vel < max_velocity_portions) cont_smooth_vel++;
    } else {
        if (cont_smooth_vel > 0) cont_smooth_vel--;
    }
    ROS_WARN_STREAM("cont smooth: " << cont_smooth_vel);

    if (d_to_target < 1) {
        cont_smooth_vel = 0;
        init_d_to_target = 0;
        cont_init_d_to_target = 0;
        next_wp = true;
    }

    output_vel.header.frame_id = "map";
    return output_vel;
}

geometry_msgs::TwistStamped calculateVelocity(geometry_msgs::PoseStamped pose, geometry_msgs::Point target) {
    double cruising_speed = 1.0;
    geometry_msgs::TwistStamped output_vel;
    geometry_msgs::Point current = pose.pose.position;

    Eigen::Vector3f x0 = Eigen::Vector3f(current.x, current.y, current.z);
    Eigen::Vector3f x2 = Eigen::Vector3f(target.x, target.y, target.z);
    // Eigen::Vector3f x1 = Eigen::Vector3f(initial.x, initial.y, initial.z); for pure pursuit ??

    double d_to_target = (x2 - x0).norm();
    Eigen::Vector3f unit_vec = (x2 - x0) / d_to_target;

    if (d_to_target >= 1) {
        output_vel.twist.linear.x = unit_vec(0) * cruising_speed;
        output_vel.twist.linear.y = unit_vec(1) * cruising_speed;
        output_vel.twist.linear.z = unit_vec(2) * cruising_speed;
        next_wp = false;
    } else {
        output_vel.twist.linear.x = 0;
        output_vel.twist.linear.y = 0;
        output_vel.twist.linear.z = 0;
        next_wp = true;
    }
    output_vel.header.frame_id = "uav_1_home";
    return output_vel;
}

int main(int _argc, char** _argv) {
    ros::init(_argc, _argv, "plot_ual");
    ros::NodeHandle nh;
    ros::Publisher pub_uav_path_actual = nh.advertise<nav_msgs::Path>("uav_path_actual", 1);
    ros::Publisher pub_uav_path_intendeed = nh.advertise<nav_msgs::Path>("uav_path_intendeed", 1);
    ros::Publisher pub_velocity = nh.advertise<geometry_msgs::TwistStamped>("/uav_1/ual/set_velocity", 1);

    grvc::ual::UAL ual(_argc, _argv);

    int uav_id;
    ros::param::param<int>("~uav_id", uav_id, 1);

    while (!ual.isReady() && ros::ok()) {
        ROS_WARN("UAL %d not ready!", uav_id);
        sleep(1);
    }
    ROS_INFO("UAL %d ready!", uav_id);

    double flight_level = 2.0;
    ual.takeOff(flight_level, true);

    uav_path_intendeed.header.frame_id = "uav_1_home";
    uav_path_actual.header.frame_id = "uav_1_home";
    geometry_msgs::PoseStamped intendeed_wp;
    intendeed_wp.pose.position.x = 0.0;
    intendeed_wp.pose.position.y = 0.0;
    intendeed_wp.pose.position.z = 2.0;
    uav_path_intendeed.poses.push_back(intendeed_wp);
    intendeed_wp.pose.position.x = 0.0;
    intendeed_wp.pose.position.y = 10.0;
    intendeed_wp.pose.position.z = 10.0;
    uav_path_intendeed.poses.push_back(intendeed_wp);
    // intendeed_wp.pose.position.x = 10.0;
    // intendeed_wp.pose.position.y = 0.0;
    // intendeed_wp.pose.position.z = 3.0;
    // uav_path_intendeed.poses.push_back(intendeed_wp);
    ros::Rate rate(10);
    while (ros::ok()) {
        grvc::ual::Waypoint wp;
        wp.header = uav_path_intendeed.poses.at(1).header;
        wp.pose = uav_path_intendeed.poses.at(1).pose;
        uav_path_actual.poses.push_back(ual.pose());
        pub_uav_path_actual.publish(uav_path_actual);
        pub_uav_path_intendeed.publish(uav_path_intendeed);
        ual.goToWaypoint(wp, false);
        ros::spinOnce();
        rate.sleep();
        // for (int i = 1; i < uav_path_intendeed.poses.size(); i++) {
        // while (!next_wp) {
        //     geometry_msgs::TwistStamped velocity_to_pub = calculateSmoothVelocity(ual.pose(), uav_path_intendeed.poses.at(i).pose.position);
        //     uav_path_actual.poses.push_back(ual.pose());
        //     pub_velocity.publish(velocity_to_pub);
        //     pub_uav_path_actual.publish(uav_path_actual);
        //     pub_uav_path_intendeed.publish(uav_path_intendeed);
        //     ros::spinOnce();
        //     rate.sleep();
        // }
        // grvc::ual::Waypoint hover;
        // hover.header = uav_path_intendeed.poses.at(i).header;
        // hover.pose = uav_path_intendeed.poses.at(i).pose;
        // uav_path_actual.poses.push_back(ual.pose());
        // ROS_WARN("Goal!");
        // ual.goToWaypoint(hover, false);
        // ros::Duration(10).sleep();
        // ROS_WARN("Go to next WP");
        // next_wp = false;
        // }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}