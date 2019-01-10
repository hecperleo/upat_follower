#include <ros/ros.h>

#include <uav_abstraction_layer/ual.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"
// linealInterp1
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

class ManagerSimple {
   public:
    ManagerSimple();
    ~ManagerSimple();

    nav_msgs::Path createPathInterp1(std::vector<double> list_x, std::vector<double> list_y, std::vector<double> list_z, int path_size, int new_path_size);

   private:
    // Callbacks
    void initPathCallback(const nav_msgs::Path &_init_path);
    void modeCallback(std_msgs::Int8 _mode);
    // Methods
    void pathManagement(std::vector<double> list_pose_x, std::vector<double> list_pose_y, std::vector<double> list_pose_z);
    int nearestNeighbourIndex(std::vector<double> &x, double &value);
    std::vector<double> linealInterp1(std::vector<double> &x, std::vector<double> &y, std::vector<double> &x_new);
    std::vector<double> interpWaypointList(std::vector<double> list_pose_axis, int amount_of_points);
    nav_msgs::Path constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z);
    // Node handlers
    ros::NodeHandle n;
    // Subscribers
    ros::Subscriber sub_path, sub_mode;
    // Publishers
    ros::Publisher pub_output_path;
    // Variables
    nav_msgs::Path output_path_;
    bool flag_sub_path = true;
    // std::vector<double> list_pose_x, list_pose_y, list_pose_z;
    enum mode_t { mode_interp1, mode_cubic_spline, mode_idle};
    mode_t mode = mode_idle;
};