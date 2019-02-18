#include <ros/package.h>
#include <ros/ros.h>
#include <uav_path_manager/FollowPath.h>
#include <uav_path_manager/GeneratePath.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

class Visualization {
   public:
    Visualization();
    ~Visualization();

    void pubMsgs();

   private:
    // Callbacks

    // Methods

    // Node handlers
    ros::NodeHandle nh_, pnh_;
    // Subscribers
    // ros::Subscriber
    // Publishers
    // ros::Publisher
    // Services
    // Variables
    // Params
};