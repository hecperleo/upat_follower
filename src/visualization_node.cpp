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
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, sEXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include <sys/stat.h>
#include <upat_follower/visualization.h>

int main(int _argc, char **_argv) {
    ros::init(_argc, _argv, "visualization_node");

    bool save_experiment_data, trajectory;
    int generator_mode;
    ros::param::param<bool>("~save_experiment_data", save_experiment_data, false);
    ros::param::param<bool>("~trajectory", trajectory, true);
    ros::param::param<int>("~generator_mode", generator_mode, 0);

    Visualization visual;
    visual.save_experiment = save_experiment_data;
    if (visual.save_experiment) {
        std::string pkg_name_path = ros::package::getPath("upat_follower");
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        std::string folder_data_name = pkg_name_path + "/data/log/" + oss.str();
        if (mkdir((folder_data_name).c_str(), 0777) == -1) ROS_WARN("Directory creation failed");
        if (trajectory) {
            visual.csv_normal_distances_.open(folder_data_name + "/normal_dist_trajectory.csv");
            visual.csv_current_path_.open(folder_data_name + "/current_trajectory.csv");
        } else {
            switch (generator_mode) {
                case 0:
                    visual.csv_normal_distances_.open(folder_data_name + "/normal_dist_linear_interp.csv");
                    visual.csv_current_path_.open(folder_data_name + "/current_path_linear_interp.csv");
                    break;
                case 1:
                    visual.csv_normal_distances_.open(folder_data_name + "/normal_dist_cubic_loyal_spline.csv");
                    visual.csv_current_path_.open(folder_data_name + "/current_path_cubic_loyal_spline.csv");
                    break;
                case 2:
                    visual.csv_normal_distances_.open(folder_data_name + "/normal_dist_cubic_spline.csv");
                    visual.csv_current_path_.open(folder_data_name + "/current_path_cubic_spline.csv");
                    break;
            }
        }
    }
    double start_time;
    bool do_once = true;
    ros::Rate rate(50);
    while (ros::ok()) {
        visual.pubMsgs();
        if (visual.current_path_.poses.size() > 0) {
            if (visual.ual_state_.state == 4) {
                if (visual.save_experiment == true) {
                    visual.saveMissionData();
                }
                if (do_once) {
                    start_time = ros::Time::now().toSec();
                    do_once = false;
                }
            } else {
                if (!do_once) {
                    ROS_INFO("Time: %f", ros::Time::now().toSec() - start_time);
                    do_once = true;
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    if (visual.save_experiment) {
        for (int i = 0; visual.current_path_.poses.size(); i++) {
            visual.csv_current_path_
                << visual.current_path_.poses.at(i).pose.position.x << ", "
                << visual.current_path_.poses.at(i).pose.position.y << ", "
                << visual.current_path_.poses.at(i).pose.position.z << std::endl;
        }
        visual.csv_normal_distances_.close();
        visual.csv_current_path_.close();
    }

    return 0;
}