#include <path_follower/manager_simple.h>

ManagerSimple::ManagerSimple() {
    n = ros::NodeHandle();
    // Subscriptions
    sub_path = n.subscribe("initPath", 0, &ManagerSimple::initPathCallback, this);
    sub_mode = n.subscribe("manager_mode", 0, &ManagerSimple::modeCallback, this);
    // Publishers
    pub_output_path = n.advertise<nav_msgs::Path>("output_path", 1000);
}

ManagerSimple::~ManagerSimple() {
}

int ManagerSimple::nearestNeighbourIndex(std::vector<double> &x, double &value) {
    double dist = std::numeric_limits<double>::max();
    double newDist = dist;
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

std::vector<double> ManagerSimple::linealInterp1(std::vector<double> &x, std::vector<double> &y, std::vector<double> &x_new) {
    std::vector<double> y_new;
    double dx, dy, m, b;
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

void ManagerSimple::modeCallback(std_msgs::Int8 _mode) {
    switch (_mode.data) {
        case 1:
            mode = mode_interp1;
            break;
        case 2:
            mode = mode_spline;
            break;
        default:
            break;
    }
    return;
}

void ManagerSimple::initPathCallback(const nav_msgs::Path &_init_path) {
    if (_init_path.poses.size() > 1 && mode != mode_idle) {
        if (flag_sub_path == true) {
            for (int i = 0; i < _init_path.poses.size(); i++) {
                list_pose_x.push_back(_init_path.poses.at(i).pose.position.x);
                list_pose_y.push_back(_init_path.poses.at(i).pose.position.y);
                list_pose_z.push_back(_init_path.poses.at(i).pose.position.z);
            }
            flag_sub_path = false;
        }
        pathManagement();
    }
    return;
}

std::vector<double> ManagerSimple::interpWaypointList(std::vector<double> list_pose_axis, int amount_of_points) {
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
    auto interp1_path = linealInterp1(aux_axis, list_pose_axis, new_aux_axis);
    return interp1_path;
}

nav_msgs::Path ManagerSimple::constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z) {
    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses(wps_x.size());
    path_msg.header.frame_id = "uav_1_home";
    for (int i = 0; i < wps_x.size(); i++) {
        poses.at(i).pose.position.x = wps_x[i];
        poses.at(i).pose.position.y = wps_y[i];
        poses.at(i).pose.position.z = wps_z[i];
        poses.at(i).pose.orientation.x = 0;
        poses.at(i).pose.orientation.y = 0;
        poses.at(i).pose.orientation.z = 0;
        poses.at(i).pose.orientation.w = 1;
    }
    path_msg.poses = poses;
    return path_msg;
}

nav_msgs::Path ManagerSimple::createPathInterp1(std::vector<double> list_x, std::vector<double> list_y, std::vector<double> list_z, int path_size, int interp1_final_size) {
    nav_msgs::Path interp1_path;
    std::vector<double> new_list_x, new_list_y, new_list_z;
    if (path_size > 1) {
        new_list_x = interpWaypointList(list_x, interp1_final_size);
        new_list_y = interpWaypointList(list_y, interp1_final_size);
        new_list_z = interpWaypointList(list_z, interp1_final_size);
        interp1_path = constructPath(new_list_x, new_list_y, new_list_z);
    }
    return interp1_path;
}

void ManagerSimple::pathManagement() {
    int interp1_final_size = 10000;
    switch (mode) {
        case mode_interp1:
            output_path_ = createPathInterp1(list_pose_x, list_pose_y, list_pose_z, list_pose_x.size(), interp1_final_size);
            break;
        case mode_spline:
            // output_path_ =
            break;
        default:
            // output_path_ =
            break;
    }
    pub_output_path.publish(output_path_);
    sleep(1);
    return;
}