#include <path_follower/initiator.h>

Initiator::Initiator() {
    n = ros::NodeHandle();

    // Publishers
    pub_path = n.advertise<nav_msgs::Path>("initPath", 1000);
    pub_vectorT = n.advertise<nav_msgs::Path>("vectorT", 1000);
    pub_vectorT_simple = n.advertise<nav_msgs::Path>("vectorT_simple", 1000);

    loop();
}

Initiator::~Initiator() {
}

void Initiator::defaultVectorT() {
    std::vector<grvc::ual::Waypoint> tList;
    grvc::ual::Waypoint t;
    if (flag_vectorT == true) {
        for (int i = 0; i < vectorT.size(); i++) {
            t.pose.position.x = vectorT[i];
            tList.push_back(t);
        }
        std::vector<geometry_msgs::PoseStamped> times(tList.size());
        for (int p = 0; p < tList.size(); p++) {
            times.at(p).pose.position.x = tList[p].pose.position.x;
        }
        msg_vectorT.poses = times;
        flag_vectorT = false;
        std::cout << "[ TEST] Vector T size  = " << msg_vectorT.poses.size() << '\n';
    }
}

void Initiator::defaultVectorT_simple() {
    std::vector<grvc::ual::Waypoint> tList_simple;
    grvc::ual::Waypoint t_simple;
    if (flag_vectorT_simple == true) {
        for (int i = 0; i < vectorT.size(); i++) {
            t_simple.pose.position.x = vectorT_simple[i];
            tList_simple.push_back(t_simple);
        }
        std::vector<geometry_msgs::PoseStamped> times_simple(tList_simple.size());
        for (int p = 0; p < tList_simple.size(); p++) {
            times_simple.at(p).pose.position.x = tList_simple[p].pose.position.x;
        }
        msg_vectorT_simple.poses = times_simple;
        flag_vectorT_simple = false;
        std::cout << "[ TEST] Vector T size  = " << msg_vectorT_simple.poses.size() << '\n';
    }
}

void Initiator::defaultPath() {
    float mult_wp = 1.0;
    std::vector<grvc::ual::Waypoint> waypointList;
    grvc::ual::Waypoint waypoint;
    msg_path.header.frame_id = "map";
    if (flag_path == true) {
        waypoint.pose.position.x = 5.0 * mult_wp;
        waypoint.pose.position.y = 5.0 * mult_wp;
        waypoint.pose.position.z = 10.01 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 5.0 * mult_wp;
        waypoint.pose.position.y = 10.0 * mult_wp;
        waypoint.pose.position.z = 10.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 5.0 * mult_wp;
        waypoint.pose.position.y = 10.0 * mult_wp;
        waypoint.pose.position.z = 5.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 5.0 * mult_wp;
        waypoint.pose.position.y = 5.0 * mult_wp;
        waypoint.pose.position.z = 5.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 10.0 * mult_wp;
        waypoint.pose.position.y = 5.0 * mult_wp;
        waypoint.pose.position.z = 5.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 10.0 * mult_wp;
        waypoint.pose.position.y = 5.0 * mult_wp;
        waypoint.pose.position.z = 10.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 10.0 * mult_wp;
        waypoint.pose.position.y = 10.0 * mult_wp;
        waypoint.pose.position.z = 10.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 15.0 * mult_wp;
        waypoint.pose.position.y = 10.0 * mult_wp;
        waypoint.pose.position.z = 10.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 15.0 * mult_wp;
        waypoint.pose.position.y = 10.0 * mult_wp;
        waypoint.pose.position.z = 5.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 15.0 * mult_wp;
        waypoint.pose.position.y = 5.0 * mult_wp;
        waypoint.pose.position.z = 5.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 20.0 * mult_wp;
        waypoint.pose.position.y = 5.0 * mult_wp;
        waypoint.pose.position.z = 5.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 20.0 * mult_wp;
        waypoint.pose.position.y = 5.0 * mult_wp;
        waypoint.pose.position.z = 10.0 * mult_wp;
        waypointList.push_back(waypoint);
        waypoint.pose.position.x = 20.0 * mult_wp;
        waypoint.pose.position.y = 10.0 * mult_wp;
        waypoint.pose.position.z = 10.0 * mult_wp;
        waypointList.push_back(waypoint);

        std::cout << "[ TEST] Running!" << '\n';
        flag_path = false;
    }
    std::vector<geometry_msgs::PoseStamped> poses(waypointList.size());
    for (int p = 0; p < waypointList.size(); p++) {
        poses.at(p).pose.position.x = waypointList[p].pose.position.x;
        poses.at(p).pose.position.y = waypointList[p].pose.position.y;
        poses.at(p).pose.position.z = waypointList[p].pose.position.z;
    }
    msg_path.poses = poses;
    std::cout << "[ TEST] Vector WP size = " << msg_path.poses.size() << '\n';
}

void Initiator::loop() {
    defaultPath();
    defaultVectorT();
    while (ros::ok()) {
        pub_path.publish(msg_path);
        pub_vectorT.publish(msg_vectorT);
        pub_vectorT_simple.publish(msg_vectorT_simple);
        sleep(0.1);
        ros::spinOnce();
    }
}