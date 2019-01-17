#include <ros/ros.h>
#include <uav_abstraction_layer/Land.h>
#include <uav_abstraction_layer/State.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/ual.h>
#include <Eigen/Eigen>
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
    void velocityCallback(const geometry_msgs::TwistStamped &_velocity);
    // Methods

    // Node handlers
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber sub_pose, sub_path, sub_state, sub_velocity;
    // Publishers
    ros::Publisher pub_set_velocity, pub_set_pose;
    // Services
    ros::ServiceClient srvTakeOff, srvLand;

    // Variables
    bool on_path, end_path;
    nav_msgs::Path path;
    geometry_msgs::PoseStamped ual_pose;
    geometry_msgs::TwistStamped velocity_;
    uav_abstraction_layer::State ual_state;

    // Params
};