#include <ros/ros.h>
#include <uav_abstraction_layer/ual.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

class Initiator {
   public:
    Initiator();
    ~Initiator();

    void loop();

   private:
    // Callbacks

    //
    void defaultPath();
    void defaultVectorT();

    // Node handlers
    ros::NodeHandle n;

    // Subscribers

    // Publishers
    ros::Publisher pubPath, pubVectorT;

    // Variables
    //std::vector<double> vectorT = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    // std::vector<double> vectorT = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0 /*, 2.0*/};
    //std::vector<double> vectorT = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};
    std::vector<double> vectorT = {2.0, 3.0, 2.0, 1.0, 2.0, 3.0, 2.0, 1.0, 2.0, 3.0, 2.0, 1.0 /*, 2.0*/};
    // std::vector<double> vectorT = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};
    double flight_level = 10.0;
    bool flagPath = true;
    bool flagVectorT = true;

    nav_msgs::Path msgPath, msgVectorT;
    // Params
};