#include <ros/ros.h>

#include <tf/tf.h>
#include <uav_abstraction_layer/ual.h>
#include <fstream>
#include "ecl/geometry.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
// Para el interp1
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

class Manager {
   public:
    Manager();
    ~Manager();

    void loop();

   private:
    // Callbacks
    // void matrix_cb(const mapping::vectorVector input);

    //
    void params();
    void UALPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg1);
    void UALPathCallback(const nav_msgs::Path &msg);
    void vectorTCallback(const nav_msgs::Path &msg);
    void spline();
    void eclSpline(float minT, float dist_total);
    void interpVectorT(int splineSize);
    void preProcessing();
    void checkTimes();
    float distance2Points(float x1, float x2, float y1, float y2, float z1, float z2);
    bool createOtherSpline(std::vector<double> vVz, int splineSize, bool safe);
    std::vector<double> increaseVector(std::vector<double> vect, int finalSize);
    std::vector<double> interpWaypoints(std::vector<double> wp, double t);
    template <typename Real>
    int nearestNeighbourIndex(std::vector<Real> &x, Real &value);
    template <typename Real>
    std::vector<Real> interp1(std::vector<Real> &x, std::vector<Real> &y, std::vector<Real> &x_new);
    nav_msgs::Path constructPath(double *x, double *y, double *z, int length);

    // Node handlers
    ros::NodeHandle n;

    // Subscribers
    ros::Subscriber sub_pose, sub_path, sub_vectorT;

    // Publishers
    ros::Publisher pub_draw_path, pub_ecl_path, pub_ecl_path_v, pub_ecl_path1, pub_ecl_path2, pub_new_vectorT;

    // Variables
    float t;
    int i = 0;
    float sum_vectorT = 0;
    const float max_velocity = 1;
    bool flag_spline = true;
    bool flag_sub_path = true;
    bool flag_sub_vectorT = true;
    bool flag_last_one = false;
    bool flag_finish_spline = false;
    double current_x, current_y, current_z;

    grvc::ual::Waypoint waypoint;
    std::vector<grvc::ual::Waypoint> aux_vector;
    std::vector<double> list_pose_x, list_pose_y, list_pose_z;
    std::vector<double> new_list_pose_x, new_list_pose_y, new_list_pose_z;
    std::vector<double> vectorT, new_vectorT, last_spline_vectorT;
    nav_msgs::Path msg_draw_path, msg_new_vectorT, path, smoothed_path, spline_path, spline_msg, splineV_msg, spline_msg1, spline_msg2;

    // Params
};