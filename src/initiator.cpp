#include <path_follower/initiator.h>

Initiator::Initiator() {
    n = ros::NodeHandle();

    // Publishers
    pubPath = n.advertise<nav_msgs::Path>("initPath", 1000);
    pubVectorT = n.advertise<nav_msgs::Path>("vectorT", 1000);

    loop();
}

Initiator::~Initiator() {
}

void Initiator::defaultVectorT() {
    std::vector<grvc::ual::Waypoint> tList;
    grvc::ual::Waypoint t;
    if (flagVectorT == true) {
        for (int i = 0; i < vectorT.size(); i++) {
            t.pose.position.x = vectorT[i];
            tList.push_back(t);
        }
        std::vector<geometry_msgs::PoseStamped> times(tList.size());
        for (int p = 0; p < tList.size(); p++) {
            times.at(p).pose.position.x = tList[p].pose.position.x;
        }
        msgVectorT.poses = times;
        flagVectorT = false;
        std::cout << "[ TEST] Vector T size  = " << msgVectorT.poses.size() << '\n';
    }
}

void Initiator::defaultPath() {
    float mult_wp = 1.0;
    std::vector<grvc::ual::Waypoint> waypointList;
    grvc::ual::Waypoint waypoint;
    msgPath.header.frame_id = "map";
    if (flagPath == true) {
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
        flagPath = false;
    }
    std::vector<geometry_msgs::PoseStamped> poses(waypointList.size());
    for (int p = 0; p < waypointList.size(); p++) {
        poses.at(p).pose.position.x = waypointList[p].pose.position.x;
        poses.at(p).pose.position.y = waypointList[p].pose.position.y;
        poses.at(p).pose.position.z = waypointList[p].pose.position.z;
    }
    msgPath.poses = poses;
    std::cout << "[ TEST] Vector WP size = " << msgPath.poses.size() << '\n';
}

void Initiator::loop() {
    defaultPath();
    defaultVectorT();
    while (ros::ok()) {
        pubPath.publish(msgPath);
        pubVectorT.publish(msgVectorT);
        sleep(0.1);
        ros::spinOnce();
    }
}