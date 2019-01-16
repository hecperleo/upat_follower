#include <ros/ros.h>
#include <uav_abstraction_layer/Land.h>
#include <uav_abstraction_layer/State.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/ual.h>
#include "ecl/geometry.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

class Manager {
   public:
    Manager();
    ~Manager();

    void runMission();

   private:
    // Callbacks
    void pathCallback(const nav_msgs::Path &_path);
    void ualStateCallback(const uav_abstraction_layer::State &_ual_state);
    void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
    // Methods

    // Node handlers
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber sub_pose, sub_path, sub_state;
    // Publishers
    ros::Publisher pub_set_velocity, pub_set_pose;
    // Services
    ros::ServiceClient srvTakeOff, srvLand;

    // Variables
    nav_msgs::Path path;
    geometry_msgs::PoseStamped ual_pose;
    uav_abstraction_layer::State ual_state;

    // Params
};