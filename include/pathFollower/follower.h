#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <uav_abstraction_layer/Land.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/SetVelocity.h>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"
#include <algorithm>
#include <fstream>
#include "ecl/containers.hpp"
#include "visualization_msgs/Marker.h"

class Follower
{

public:
  Follower();
  ~Follower();

  void mision();

private:
  // Callbacks
  void UALPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void UALVelocityCallback(const geometry_msgs::TwistStamped &msg);
  void UALPathCallback(const nav_msgs::Path &msg);
  void UALPathVCallback(const nav_msgs::Path &msg);
  void newVectorTCallback(const nav_msgs::Path &msg);

  //
  void pure_pursuit();
  void trayectoriaActual();
  void funcCambiaLookAhead(int p);
  void funcCambiaLookAheadVariable();
  void funcDrawCylinder();
  void funcDrawTriangles(int path_pos);
  float funcMod(float x1, float x2, float y1, float y2, float z1, float z2);
  float funcModDirection(float x1, float y1, float z1, float LA);
  float funcCalcDistNormal();
  float funcCalcDistNormalPos();
  float funcSumDistNormal();
  float movingAverageVx(float Vx);

  // Node handlers
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber subPose, subActualVel, subVel, subPath, subNewVectorT;
  // Publishers
  ros::Publisher pubToTarget, pubNormalDist, pubLookAhead, pubDrawPathActual, vis_pub;
  // Services
  ros::ServiceClient srvTakeOff, srvLand, srvGoToWaypoint, srvSetVelocity;
  uav_abstraction_layer::TakeOff take_off;
  uav_abstraction_layer::Land land;
  uav_abstraction_layer::GoToWaypoint go_to_waypoint;
  uav_abstraction_layer::SetVelocity set_velocity;

  // Variables
  int i = 0;
  int pos_path = 0;
  // Flags
  bool flagSubPath = true;
  bool flagSubVel = true;
  bool flagPurePursuit = true;
  bool flagSubVectorT = true;
  // Velocidades
  float actualVelX, actualVelY, actualVelZ;
  // Posiciones
  float actualPosX, actualPosY, actualPosZ;
  // Distancias
  float dist_normal, dist_toTarget, suma_distNormal, dist_normal_prev;
  // Pure pursuit
  float pos_pure_pursuit;
  // Look Ahead
  const float velocidadMax = 1;
  const float lookAhead_init = 1;
  float lookAhead = velocidadMax; // Comentar para tener el generador V1
  // float lookAhead = lookAhead_init; // Descomentar para tener el generador V1
  float prev_lookAhead = lookAhead;
  // Flight level
  const double flight_level = 5.0;
  // Paths
  std::vector<grvc::ual::Waypoint> path_ok;
  std::vector<grvc::ual::Velocity> path_v;
  // Waypoints
  grvc::ual::Waypoint origen, home;
  // Velocidades
  geometry_msgs::TwistStamped Vel_goClose;
  // Visualizar look ahead y actual path
  nav_msgs::Path pathDistToTarget, pathNormalDist, pathLookAhead, msgDrawActual;
  std::vector<grvc::ual::Waypoint> vecAux;
  // Vector de tiempos
  std::vector<double> newVectorT;
  // Markers
  visualization_msgs::Marker marker;

  // Params
};