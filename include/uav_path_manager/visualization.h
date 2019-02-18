#include <ros/package.h>
#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/Visualize.h>
#include <visualization_msgs/Marker.h>
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
    visualization_msgs::Marker readModel(std::string _model_type);
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    ros::Subscriber sub_pose_;
    // Publishers
    ros::Publisher pub_init_path_, pub_generated_path_, pub_current_path_, pub_uav_model_;
    // Services
    ros::ServiceServer server_visualize_;
    // Variables
    geometry_msgs::PoseStamped ual_pose_;
    nav_msgs::Path init_path_, generated_path_, current_path_;
    visualization_msgs::Marker uav_model_;
    // Params
    int uav_id_;
    std::string model_;
};