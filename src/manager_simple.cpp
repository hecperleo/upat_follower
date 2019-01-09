#include <path_follower/manager_simple.h>

ManagerSimple::ManagerSimple() {
    n = ros::NodeHandle();
    // Subscriptions
    sub_path = n.subscribe("initPath", 0, &ManagerSimple::UALPathCallback, this);

    // Publishers
    pub_path_interp1 = n.advertise<nav_msgs::Path>("path_interp1", 1000);

    loop();
}

ManagerSimple::~ManagerSimple() {
}

void ManagerSimple::UALPathCallback(const nav_msgs::Path &msg) {
    double poseX, poseY, poseZ;
    std::vector<grvc::ual::Waypoint> poseList;
    if (flag_sub_path == true) {
        for (auto p : msg.poses) {
            waypoint.pose.position.x = p.pose.position.x;
            waypoint.pose.position.y = p.pose.position.y;
            waypoint.pose.position.z = p.pose.position.z;
            poseList.push_back(waypoint);
            poseX = p.pose.position.x;
            list_pose_x.push_back(poseX);
            poseY = p.pose.position.y;
            list_pose_y.push_back(poseY);
            poseZ = p.pose.position.z;
            list_pose_z.push_back(poseZ);
        }
        for (int q = 0; q < poseList.size(); q++) {
            waypoint.pose.position.x = poseList[q].pose.position.x;
            waypoint.pose.position.y = poseList[q].pose.position.y;
            waypoint.pose.position.z = poseList[q].pose.position.z;
            path.poses.push_back(waypoint);
        }
    }
    flag_sub_path = false;

    return;
}

nav_msgs::Path ManagerSimple::constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z) {
    nav_msgs::Path msg;
    std::vector<geometry_msgs::PoseStamped> poses(wps_x.size());
    msg.header.frame_id = "map";
    for (int i = 0; i < wps_x.size(); i++) {
        poses.at(i).pose.position.x = wps_x[i];
        poses.at(i).pose.position.y = wps_y[i];
        poses.at(i).pose.position.z = wps_z[i];
        poses.at(i).pose.orientation.x = 0;
        poses.at(i).pose.orientation.y = 0;
        poses.at(i).pose.orientation.z = 0;
        poses.at(i).pose.orientation.w = 1;
    }
    msg.poses = poses;
    return msg;
}

std::vector<double> ManagerSimple::interpWaypoints(std::vector<double> list_pose_axis) {
    double max_aux_axis = 100000;
    std::vector<double> aux_axis;
    std::vector<double> new_list_pose;
    for (int i = 0; i < max_aux_axis; i++){ aux_axis.push_back(i);}
    double portion = (aux_axis.back() - aux_axis.front()) / (max_aux_axis);
    double new_pose = aux_axis.front();
    new_list_pose.push_back(new_pose);
    for (int i = 1; i < path.poses.size(); i++) {
        new_pose = new_pose + portion;
        new_list_pose.push_back(new_pose);
    }
    auto res = interp1(aux_axis, list_pose_axis, new_list_pose);
    return res;
}

template <typename Real>
int ManagerSimple::nearestNeighbourIndex(std::vector<Real> &x, Real &value) {
    Real dist = std::numeric_limits<Real>::max();
    Real newDist = dist;
    size_t idx = 0;

    for (size_t i = 0; i < x.size(); ++i) {
        newDist = std::abs(value - x[i]);
        if (newDist <= dist) {
            dist = newDist;
            idx = i;
        }
    }

    return idx;
}

template <typename Real>
std::vector<Real> ManagerSimple::interp1(std::vector<Real> &x, std::vector<Real> &y, std::vector<Real> &x_new) {
    std::vector<Real> y_new;
    Real dx, dy, m, b;
    size_t x_max_idx = x.size() - 1;
    size_t x_new_size = x_new.size();

    y_new.reserve(x_new_size);

    for (size_t i = 0; i < x_new_size; ++i) {
        size_t idx = nearestNeighbourIndex(x, x_new[i]);

        if (x[idx] > x_new[i]) {
            dx = idx > 0 ? (x[idx] - x[idx - 1]) : (x[idx + 1] - x[idx]);
            dy = idx > 0 ? (y[idx] - y[idx - 1]) : (y[idx + 1] - y[idx]);
        } else {
            dx = idx < x_max_idx ? (x[idx + 1] - x[idx]) : (x[idx] - x[idx - 1]);
            dy = idx < x_max_idx ? (y[idx + 1] - y[idx]) : (y[idx] - y[idx - 1]);
        }

        m = dy / dx;
        b = y[idx] - x[idx] * m;

        y_new.push_back(x_new[i] * m + b);
    }

    return y_new;
}

void ManagerSimple::loop() {
    nav_msgs::Path path_interp1;
    while (ros::ok()) {
        if (path.poses.size() > 1 && flag_spline == true) {
            new_list_pose_x = interpWaypoints(list_pose_x);
            new_list_pose_y = interpWaypoints(list_pose_y);
            new_list_pose_z = interpWaypoints(list_pose_z);
            path_interp1 = constructPath(new_list_pose_x, new_list_pose_y, new_list_pose_z);
            flag_spline = false;
        std::cout << "size path: " << path_interp1.poses.size() << '\n';
        }
        // Publish path
        pub_path_interp1.publish(path_interp1);
        sleep(0.1);
        ros::spinOnce();
    }
}