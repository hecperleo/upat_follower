#include <uav_abstraction_layer/ual.h>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"

class PathFollower {
   public:
    PathFollower();
    ~PathFollower();

   private:
    // Callbacks

    // Methods

    // Node handlers
    ros::NodeHandle nh;

    // Subscribers

    // Publishers
};