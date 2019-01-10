#include <path_follower/manager_simple.h>

ManagerSimple::ManagerSimple() {
    n = ros::NodeHandle();
    // Subscriptions
    sub_path = n.subscribe("initPath", 0, &ManagerSimple::UALPathCallback, this);
    // Publishers
    pub_path_interp1 = n.advertise<nav_msgs::Path>("path_interp1", 1000);
}

ManagerSimple::~ManagerSimple() {
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

void ManagerSimple::UALPathCallback(const nav_msgs::Path &msg) {
    if (flag_sub_path == true && msg.poses.size() > 1) {
        for (int i = 0; i < msg.poses.size(); i++) {
            list_pose_x.push_back(msg.poses.at(i).pose.position.x);
            list_pose_y.push_back(msg.poses.at(i).pose.position.y);
            list_pose_z.push_back(msg.poses.at(i).pose.position.z);
        }
        int new_path_size = 10000;
        path_interp1 = smoothingInterp1(list_pose_x, list_pose_y, list_pose_z, list_pose_x.size(), new_path_size);
    }
    flag_sub_path = false;
    pub_path_interp1.publish(path_interp1);
    sleep(1);
    return;
}

nav_msgs::Path ManagerSimple::constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z) {
    nav_msgs::Path msg;
    std::vector<geometry_msgs::PoseStamped> poses(wps_x.size());
    msg.header.frame_id = "uav_1_home";
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

std::vector<double> ManagerSimple::interpWaypoints(std::vector<double> list_pose_axis, int amount_of_points) {
    std::vector<double> aux_axis;
    std::vector<double> new_aux_axis;
    for (int i = 0; i < list_pose_axis.size(); i++) {
        aux_axis.push_back(i);
    }
    double portion = (aux_axis.back() - aux_axis.front()) / (amount_of_points);
    double new_pose = aux_axis.front();
    new_aux_axis.push_back(new_pose);
    for (int i = 1; i < amount_of_points; i++) {
        new_pose = new_pose + portion;
        new_aux_axis.push_back(new_pose);
    }
    auto res = interp1(aux_axis, list_pose_axis, new_aux_axis);
    return res;
}


nav_msgs::Path ManagerSimple::smoothingInterp1(std::vector<double> list_x, std::vector<double> list_y, std::vector<double> list_z, int path_size, int new_path_size) {
    nav_msgs::Path res;
    std::vector<double> new_list_x, new_list_y, new_list_z;
    if (path_size > 1) {
        new_list_x = interpWaypoints(list_x, new_path_size);
        new_list_y = interpWaypoints(list_y, new_path_size);
        new_list_z = interpWaypoints(list_z, new_path_size);
        res = constructPath(new_list_x, new_list_y, new_list_z);
    }
    return res;
}