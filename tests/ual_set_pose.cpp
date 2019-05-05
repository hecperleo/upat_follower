//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include <upat_follower/Visualize.h>
#include <Eigen/Eigen>

int main(int _argc, char** _argv) {
    grvc::ual::UAL ual(_argc, _argv);
    while ((ual.state().state != uav_abstraction_layer::State::LANDED_ARMED) && ros::ok()) {
        std::cout << "UAL not ready!" << std::endl;
        sleep(1);
    }
    ros::NodeHandle nh_;
    ros::ServiceClient client_visualize_ = nh_.serviceClient<upat_follower::Visualize>("/upat_follower/visualization/uav_1/visualize");

    // TODO(franreal): rename file to test_ual_class?
    // Use class interface to perform a simple mission:
    std::cout << "Mission using UAL object" << std::endl;
    // Define flight level and take off
    double flight_level = 12.5;
    ual.takeOff(flight_level);

    // Define path (ugly due to lack of constructor)
    nav_msgs::Path init_path, current_path;
    init_path.header.frame_id = current_path.header.frame_id = "uav_1_home";
    geometry_msgs::PoseStamped waypoint;
    waypoint.header.frame_id = "uav_1_home";
    waypoint.pose.position.x = 0.0;
    waypoint.pose.position.y = 0.0;
    waypoint.pose.position.z = 12.5;
    waypoint.pose.orientation.x = 0;
    waypoint.pose.orientation.y = 0;
    waypoint.pose.orientation.z = 0;
    waypoint.pose.orientation.w = 1;
    init_path.poses.push_back(waypoint);
    waypoint.pose.position.x = 30.0;
    waypoint.pose.position.y = 30.0;
    waypoint.pose.position.z = 12.5;
    init_path.poses.push_back(waypoint);
    // waypoint.pose.position.x = 5.0;
    // waypoint.pose.position.y = 5.0;
    // waypoint.pose.position.z = 5.0;
    // init_path.poses.push_back(waypoint);
    // waypoint.pose.position.x = 5.0;
    // waypoint.pose.position.y = -2.5;
    // waypoint.pose.position.z = 5.0;
    // init_path.poses.push_back(waypoint);
    // waypoint.pose.position.x = 12.5;
    // waypoint.pose.position.y = -2.5;
    // waypoint.pose.position.z = 5.0;
    // init_path.poses.push_back(waypoint);
    // waypoint.pose.position.x = 12.5;
    // waypoint.pose.position.y = -2.5;
    // waypoint.pose.position.z = 5.0;
    // init_path.poses.push_back(waypoint);
    // waypoint.pose.position.x = 12.5;
    // waypoint.pose.position.y = -2.5;
    // waypoint.pose.position.z = 12.5;
    // init_path.poses.push_back(waypoint);
    // waypoint.pose.position.x = 12.5;
    // waypoint.pose.position.y = 5.0;
    // waypoint.pose.position.z = 12.5;
    // init_path.poses.push_back(waypoint);
    // waypoint.pose.position.x = 20.0;
    // waypoint.pose.position.y = 5.0;
    // waypoint.pose.position.z = 12.5;
    // init_path.poses.push_back(waypoint);
    // waypoint.pose.position.x = 20.0;
    // waypoint.pose.position.y = 5.0;
    // waypoint.pose.position.z = 5.0;
    // init_path.poses.push_back(waypoint);

    upat_follower::Visualize visualize;
    visualize.request.init_path = init_path;
    client_visualize_.call(visualize);

    ual.goToWaypoint(init_path.poses.front(), true);

    for (auto p : init_path.poses) {
        std::cout << "Waypoint: " << p.pose.position.x << ", " << p.pose.position.y << ", " << p.pose.position.z << ", frame_id: " << p.header.frame_id << std::endl;
        ual.setPose(p);
        Eigen::Vector3f p1, p2;
        p1 = Eigen::Vector3f(ual.pose().pose.position.x, ual.pose().pose.position.y, ual.pose().pose.position.z);
        p2 = Eigen::Vector3f(p.pose.position.x, p.pose.position.y, p.pose.position.z);
        while ((p2 - p1).norm() > 0.5) {
            p1 = Eigen::Vector3f(ual.pose().pose.position.x, ual.pose().pose.position.y, ual.pose().pose.position.z);
            p2 = Eigen::Vector3f(p.pose.position.x, p.pose.position.y, p.pose.position.z);
            current_path.poses.push_back(ual.pose());
            visualize.request.current_path = current_path;
            client_visualize_.call(visualize);
            sleep(0.1);
        }
        std::cout << "Arrived!" << std::endl;
    }

    // Land
    ual.land();

    return 0;
}
