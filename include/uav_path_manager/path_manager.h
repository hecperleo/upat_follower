#include <ros/ros.h>
#include <uav_abstraction_layer/Land.h>
#include <uav_abstraction_layer/State.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/GeneratePath.h>
#include <uav_path_manager/GetGeneratedPath.h>
#include <Eigen/Eigen>
#include <fstream>
#include "ecl/geometry.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"
#include <ros/package.h>

class PathManager {
   public:
    PathManager();
    ~PathManager();

    void runMission();
    void pubMsgs();

   private:
    // Callbacks
    void ualStateCallback(const uav_abstraction_layer::State &_ual_state);
    void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
    void velocityCallback(const geometry_msgs::TwistStamped &_velocity);
    // Methods
    nav_msgs::Path constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z, std::string frame_id);
    void saveDataForTesting();
    // Node handlers
    ros::NodeHandle nh, pnh;
    // Subscribers
    ros::Subscriber sub_pose, sub_state, sub_velocity;
    // Publishers
    ros::Publisher pub_generated_path, pub_init_path, pub_current_path, pub_set_velocity, pub_set_pose;
    // Services
    ros::ServiceClient srv_take_off, srv_land, srv_generated_path, srv_give_generated_path;
    // Variables
    std::string folder_data_name;
    bool on_path, end_path;
    nav_msgs::Path path, init_path, current_path;
    geometry_msgs::PoseStamped ual_pose;
    geometry_msgs::TwistStamped velocity_;
    uav_abstraction_layer::State ual_state;
    std::vector<double> list_init_x = {5.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0, 15.0, 15.0, 15.0, 20.0, 20.0, 20.0, 20.0};  // Last waypoint
    std::vector<double> list_init_y = {-2.5, 2.5, 2.5, -2.5, -2.5, -2.5, 2.5, 2.5, 2.5, -2.5, -2.5, -2.5, 2.5, 2.5};     // should be
    std::vector<double> list_init_z = {10.0, 10.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0};    // duplicated
    // Params
    int uav_id;
    bool save_csv;
};