//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 Hector Perez Leon
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

#include <upat_follower/ual_communication.h>


nav_msgs::Path constructPath(std::vector<double> &_wps_x, std::vector<double> &_wps_y, std::vector<double> &_wps_z, std::string frame_id) {
    nav_msgs::Path out_path;
    std::vector<geometry_msgs::PoseStamped> poses(_wps_x.size());
    out_path.header.frame_id = frame_id;
    for (int i = 0; i < _wps_x.size(); i++) {
        poses.at(i).pose.position.x = _wps_x[i];
        poses.at(i).pose.position.y = _wps_y[i];
        poses.at(i).pose.position.z = _wps_z[i];
        poses.at(i).pose.orientation.x = 0;
        poses.at(i).pose.orientation.y = 0;
        poses.at(i).pose.orientation.z = 0;
        poses.at(i).pose.orientation.w = 1;
    }
    out_path.poses = poses;
    return out_path;
}

nav_msgs::Path csvToPath(std::string _file_name) {
    nav_msgs::Path out_path;
    std::string pkg_name_path = ros::package::getPath("upat_follower");
    std::string folder_name = pkg_name_path + _file_name;
    std::fstream read_csv;
    read_csv.open(folder_name);
    std::vector<double> list_x, list_y, list_z;
    if (read_csv.is_open()) {
        while (read_csv.good()) {
            std::string x, y, z;
            double dx, dy, dz;
            getline(read_csv, x, ',');
            getline(read_csv, y, ',');
            getline(read_csv, z, '\n');
            std::stringstream sx(x);
            std::stringstream sy(y);
            std::stringstream sz(z);
            sx >> dx;
            sy >> dy;
            sz >> dz;
            list_x.push_back(dx);
            list_y.push_back(dy);
            list_z.push_back(dz);
        }
    } else {
        ROS_ERROR_STREAM(folder_name << " not found!");
    }

    return constructPath(list_x, list_y, list_z, "uav_1/odom");
}

std::vector<double> csvToVector(std::string _file_name) {
    std::vector<double> out_vector;
    std::string pkg_name_path = ros::package::getPath("upat_follower");
    std::string folder_name = pkg_name_path + _file_name;
    std::fstream read_csv;
    read_csv.open(folder_name);
    if (read_csv.is_open()) {
        while (read_csv.good()) {
            std::string x;
            double dx;
            getline(read_csv, x, '\n');
            std::stringstream sx(x);
            sx >> dx;
            out_vector.push_back(dx);
        }
    }

    return out_vector;
}

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "ual_communication_node");

    upat_follower::UALCommunication ual_communication;
    int pub_rate, uav_id;
    std::string ns_prefix, init_path_name;
    ros::param::param<int>("~uav_id", uav_id, 1);
    ros::param::param<int>("~pub_rate", pub_rate, 30);
    ros::param::param<std::string>("~ns_prefix", ns_prefix, "uav_");
    ros::param::param<std::string>("path", init_path_name, "cubic");
    ros::Rate rate(pub_rate);
    ros::NodeHandle nh;
    ros::ServiceClient client_visualize = nh.serviceClient<upat_follower::Visualize>("/uav_1/upat_follower/visualization/visualize");


    static upat_follower::Follower follower(1, 0, 1, 1, 1);
    nav_msgs::Path init_path = csvToPath("/config/" + init_path_name + ".csv");
    std::vector<double> times = csvToVector("/config/" + init_path_name + "_t.csv");
    nav_msgs::Path target_path = follower.prepareTrajectory(init_path, times, 0, 1);
    upat_follower::Visualize visualize;
    visualize.request.init_path = init_path;
    visualize.request.generated_path = target_path;

    while (ros::ok()) {
        client_visualize.call(visualize);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}