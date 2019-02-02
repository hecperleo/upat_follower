#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_path_manager/GetGeneratedPath.h>
#include <Eigen/Eigen>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"

class PathFollower {
   public:
    PathFollower();
    ~PathFollower();

    void pubMsgs();
    void followPath();

   private:
    // Callbacks
    bool pathCallback(uav_path_manager::GetGeneratedPath::Request &_req_path, uav_path_manager::GetGeneratedPath::Response &_res_path);
    void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose);
    // Methods
    int calculatePosOnPath(Eigen::Vector3f _current_p);
    int calculatePosLookAhead(int _pos_on_path);
    geometry_msgs::TwistStamped calculateVelocity(Eigen::Vector3f _current_p, int _pos_la);
    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    ros::Subscriber sub_pose_;
    // Publishers
    ros::Publisher pub_output_velocity_;
    // Services
    ros::ServiceServer srv_get_generated_path_;
    // Variables
    int uav_id_;
    bool flag_run_;
    double look_ahead_ = 1.0;
    double cruising_speed_ = 1.0;
    nav_msgs::Path target_path_;
    geometry_msgs::PoseStamped ual_pose_;
    geometry_msgs::TwistStamped out_velocity_;
};