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
    void defaultVectorT_simple();

    // Node handlers
    ros::NodeHandle n;

    // Subscribers

    // Publishers
    ros::Publisher pub_path, pub_vectorT, pub_vectorT_simple;

    // Variables
    std::vector<double> vectorT = {2.0, 3.0, 2.0, 1.0, 2.0, 3.0, 2.0, 1.0, 2.0, 3.0, 2.0, 1.0};
    std::vector<double> vectorT_simple = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    double flight_level = 10.0;
    bool flag_path = true;
    bool flag_vectorT = true;
    bool flag_vectorT_simple = true;

    nav_msgs::Path msg_path, msg_vectorT, msg_vectorT_simple;
    // Params
};