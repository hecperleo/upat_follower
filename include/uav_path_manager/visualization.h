#include <ros/package.h>
#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/Visualize.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

class Visualization {
   public:
    Visualization();
    ~Visualization();

    void pubMsgs();

   private:
    // Callbacks
    void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
    bool visualCallback(uav_path_manager::Visualize::Request &_req_visual, uav_path_manager::Visualize::Response &_res_visual);
    // Methods

    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    ros::Subscriber sub_pose_;
    // Publishers
    ros::Publisher pub_init_path_, pub_generated_path_;
    // Services
    ros::ServiceServer server_visualize_;
    // Variables
    geometry_msgs::PoseStamped ual_pose_;
    nav_msgs::Path init_path_, generated_path_;
    // Params
    int uav_id_;
};