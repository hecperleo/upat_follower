#include <uav_abstraction_layer/GoToWaypoint.h>
#include <uav_abstraction_layer/Land.h>
#include <uav_abstraction_layer/SetVelocity.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/State.h>

#include <ros/ros.h>
#include <algorithm>
#include <fstream>
#include "ecl/containers.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"

class Follower {
   public:
    Follower();
    ~Follower();

    void mision();

   private:
    // Callbacks
    void UALPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void UALStateCallback(const uav_abstraction_layer::State &msg);
    void UALVelocityCallback(const geometry_msgs::TwistStamped &msg);
    void UALPathCallback(const nav_msgs::Path &msg);
    void UALPathVCallback(const nav_msgs::Path &msg);
    void newVectorTCallback(const nav_msgs::Path &msg);

    //
    void purePursuit();
    void currentTrajectory();
    void changeLookAhead(int p);
    void changeLookAheadVariable();
    void drawCylinder();
    void drawTriangles(int path_pos);
    float distance2Points(float x1, float x2, float y1, float y2, float z1, float z2);
    float distance2PointsDirection(float x1, float y1, float z1, float LA);
    float calculateNormalDistance();
    float calculateNormalDistancePos();
    float calculateSumNormalDistance();
    float movingAverageVx(float Vx);

    // Node handlers
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber sub_pose, sub_state, sub_current_velocity, sub_velocity, sub_path, sub_new_vectorT;
    // Publishers
    ros::Publisher pub_to_target, pub_set_pose, pub_set_velocity, pub_normal_distance, pub_look_ahead, pub_draw_current_path, pub_visualization_marker;
    // Services
    ros::ServiceClient srvTakeOff, srvLand, srvGoToWaypoint, srvSetVelocity;
    uav_abstraction_layer::TakeOff take_off;
    uav_abstraction_layer::Land land;
    uav_abstraction_layer::GoToWaypoint go_to_waypoint;
    uav_abstraction_layer::SetVelocity set_velocity;

    // Variables
    // uav_abstraction_layer::State state;
    int state;
    int i = 0;
    int pos_path = 0;
    // Flags
    bool flag_sub_path = true;
    bool flag_sub_velocity = true;
    bool flag_pure_pursuit = true;
    bool flag_sub_vectorT = true;
    // Velocidades
    float current_velocity_x, current_velocity_y, current_velocity_z;
    // Posiciones
    float current_x, current_y, current_z;
    // Distancias
    float normal_distance, to_target_distance, sum_normal_distance, prev_normal_distance;
    // Pure pursuit
    float pure_pursuit_pos;
    // Look Ahead
    const float max_velocity = 1;
    const float init_look_ahead = 1;
    float look_ahead = max_velocity;  // Comentar para tener el generador V1
    // float look_ahead = init_look_ahead; // Descomentar para tener el generador V1
    float prev_look_ahead = look_ahead;
    // Flight level
    const double flight_level = 5.0;
    // Paths
    std::vector<grvc::ual::Waypoint> path_ok;
    std::vector<grvc::ual::Velocity> path_v;
    // Waypoints
    grvc::ual::Waypoint origen, home;
    // Velocidades
    geometry_msgs::TwistStamped go_close_velocity;
    // Visualizar look ahead y actual path
    nav_msgs::Path path_to_target_distance, path_normal_distance, path_look_ahead, msg_current_draw;
    std::vector<grvc::ual::Waypoint> aux_vector;
    // Vector de tiempos
    std::vector<double> new_vectorT;
    // Markers
    visualization_msgs::Marker marker;

    // Params
};