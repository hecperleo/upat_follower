#include <ros/package.h>
#include <ros/ros.h>
#include <uav_abstraction_layer/Land.h>
#include <uav_abstraction_layer/State.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/FollowPath.h>
#include <uav_path_manager/GeneratePath.h>
#include <Eigen/Eigen>
#include <fstream>
#include "ecl/geometry.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"

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
    nav_msgs::Path constructPath(std::vector<double> _wps_x, std::vector<double> _wps_y, std::vector<double> _wps_z, std::string frame_id);
    void saveDataForTesting();
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    ros::Subscriber sub_pose_, sub_state_, sub_velocity_;
    // Publishers
    ros::Publisher pub_generated_path_, pub_init_path_, pub_current_path_, pub_set_velocity_, pub_set_pose_;
    // Services
    ros::ServiceClient client_take_off_, client_land_, client_generate_path_, client_follow_path_;
    // Variables
    std::string folder_data_name_;
    bool on_path_, end_path_;
    nav_msgs::Path path, vel_percentage_path_, init_path_, current_path_;
    geometry_msgs::PoseStamped ual_pose_;
    geometry_msgs::TwistStamped velocity_;
    uav_abstraction_layer::State ual_state_;
    std::vector<double> list_init_x_ = {5.0, 5.0, 5.0, 5.0, 12.5, 12.5, 12.5, 20.0, 20.0};
    std::vector<double> list_init_y_ = {-2.5, 5.0, 5.0, -2.5, -2.5, -2.5, 5.0, 5.0, 5.0};
    std::vector<double> list_init_z_ = {12.5, 12.5, 5.0, 5.0, 5.0, 12.5, 12.5, 12.5, 5.0};
    // std::vector<double> list_init_x_ = {-2.50, -2.66, -20, -25.75, -25.75, -14.25, -14.25, -20.00, -2.66};
    // std::vector<double> list_init_y_ = {5.10, 5.0, 0.0, -4.0, 5.0, 5.0, -4.0, 0.0, 5.0};
    // std::vector<double> list_init_z_ = {2.25, 14.0, 14.0, 10.0, 10.0, 10.0, 10.0, 14.0, 14.0};
    std::vector<double> max_vel_percentage = {1.0, 0.8, 0.5, 0.8, 1, 0.8, 0.5, 0.8};
    // Params
    int uav_id_;
    bool save_csv_;
    bool trajectory_;
};