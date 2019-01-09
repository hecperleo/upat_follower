#include <ros/ros.h>

#include <uav_abstraction_layer/ual.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
// interp1
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

class ManagerSimple {
   public:
    ManagerSimple();
    ~ManagerSimple();

    nav_msgs::Path smoothingInterp1(std::vector<double> list_x, std::vector<double> list_y, std::vector<double> list_z, int path_size, int new_path_size);

   private:
    // Callbacks
    void UALPathCallback(const nav_msgs::Path &msg);
    // Methods
    std::vector<double> interpWaypoints(std::vector<double> list_pose_axis, int amount_of_points);
    template <typename Real>
    int nearestNeighbourIndex(std::vector<Real> &x, Real &value);
    template <typename Real>
    std::vector<Real> interp1(std::vector<Real> &x, std::vector<Real> &y, std::vector<Real> &x_new);
    nav_msgs::Path constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z);
    // Node handlers
    ros::NodeHandle n;
    // Subscribers
    ros::Subscriber sub_path;
    // Publishers
    ros::Publisher pub_path_interp1;
    // Variables
    nav_msgs::Path path_interp1;
    bool flag_sub_path = true;
    std::vector<double> list_pose_x, list_pose_y, list_pose_z;
};