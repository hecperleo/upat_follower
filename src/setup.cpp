#include <uav_abstraction_layer/ual.h>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

// std::vector<double> vectorT = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
// std::vector<double> vectorT = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
// std::vector<double> vectorT = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};
std::vector<double> vectorT = {2.0, 3.0, 2.0, 1.0, 2.0, 3.0, 2.0, 1.0, 2.0, 3.0, 2.0, 1.0, 2.0};
// std::vector<double> vectorT = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
// std::vector<double> vectorT = {2.0, 3.0, 2.0, 1.0, 2.0, 3.0, 2.0, 1.0};

double flight_level = 10.0;
bool flagPath = true;
bool flagVectorT = true;

nav_msgs::Path msgPath, msgVectorT;

void defaultPath();
void defaultVectorT();

int main(int _argc, char **_argv)
{
  ros::init(_argc, _argv, "Setup");
  ros::NodeHandle n;

  ros::Publisher pubPath =
      n.advertise<nav_msgs::Path>("initPath", 1000);
  ros::Publisher pubVectorT =
      n.advertise<nav_msgs::Path>("vectorT", 1000);

  ros::Rate loop_rate(100);
  defaultPath();
  defaultVectorT();
  while (ros::ok())
  {
    pubPath.publish(msgPath);
    pubVectorT.publish(msgVectorT);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void defaultVectorT()
{
  std::vector<grvc::ual::Waypoint> tList;
  grvc::ual::Waypoint t;
  if (flagVectorT == true)
  {
    for (int i = 0; i < vectorT.size(); i++)
    {
      t.pose.position.x = vectorT[i];
      tList.push_back(t);
    }
    std::vector<geometry_msgs::PoseStamped> times(tList.size());
    for (int p = 0; p < tList.size(); p++)
    {
      times.at(p).pose.position.x = tList[p].pose.position.x;
    }
    msgVectorT.poses = times;
    flagVectorT = false;
    std::cout << "[ TEST] Vector T size  = " << msgVectorT.poses.size() << '\n';
  }
}

void defaultPath()
{
  float mult_wp = 1;
  std::vector<grvc::ual::Waypoint> waypointList;
  grvc::ual::Waypoint waypoint;
  msgPath.header.frame_id = "map";
  if (flagPath == true)
  {
    waypoint.pose.position.x = 5.0 * mult_wp;
    waypoint.pose.position.y = 7.5 * mult_wp;
    waypoint.pose.position.z = 10.0 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 5.0 * mult_wp;
    waypoint.pose.position.y = 10.0 * mult_wp;
    waypoint.pose.position.z = 7.5 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 5.0 * mult_wp;
    waypoint.pose.position.y = 7.5 * mult_wp;
    waypoint.pose.position.z = 5.0 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 5.0 * mult_wp;
    waypoint.pose.position.y = 5.0 * mult_wp;
    waypoint.pose.position.z = 7.5 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 10.0 * mult_wp;
    waypoint.pose.position.y = 5.0 * mult_wp;
    waypoint.pose.position.z = 7.5 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 10.0 * mult_wp;
    waypoint.pose.position.y = 7.5 * mult_wp;
    waypoint.pose.position.z = 10.0 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 10.0 * mult_wp;
    waypoint.pose.position.y = 10.0 * mult_wp;
    waypoint.pose.position.z = 7.5 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 15.0 * mult_wp;
    waypoint.pose.position.y = 10.0 * mult_wp;
    waypoint.pose.position.z = 7.5 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 15.0 * mult_wp;
    waypoint.pose.position.y = 7.5 * mult_wp;
    waypoint.pose.position.z = 5.0 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 15.0 * mult_wp;
    waypoint.pose.position.y = 5.0 * mult_wp;
    waypoint.pose.position.z = 7.5 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 20.0 * mult_wp;
    waypoint.pose.position.y = 5.0 * mult_wp;
    waypoint.pose.position.z = 7.5 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 20.0 * mult_wp;
    waypoint.pose.position.y = 7.5 * mult_wp;
    waypoint.pose.position.z = 10.0 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 20.0 * mult_wp;
    waypoint.pose.position.y = 10.0 * mult_wp;
    waypoint.pose.position.z = 7.5 * mult_wp;
    waypointList.push_back(waypoint);
    waypoint.pose.position.x = 25.0 * mult_wp;
    waypoint.pose.position.y = 10.0 * mult_wp;
    waypoint.pose.position.z = 7.5 * mult_wp;
    waypointList.push_back(waypoint);

    std::cout << "[ TEST] Running!" << '\n';
    flagPath = false;
  }
  std::vector<geometry_msgs::PoseStamped> poses(waypointList.size());
  for (int p = 0; p < waypointList.size(); p++)
  {
    poses.at(p).pose.position.x = waypointList[p].pose.position.x;
    poses.at(p).pose.position.y = waypointList[p].pose.position.y;
    poses.at(p).pose.position.z = waypointList[p].pose.position.z;
  }
  msgPath.poses = poses;
  std::cout << "[ TEST] Vector WP size = " << msgPath.poses.size() << '\n';
}

/* ZigZag Plano
  waypoint.pose.position.x = 5.0*mult_wp;
  waypoint.pose.position.y = 10.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 5.0*mult_wp;
  waypoint.pose.position.y = 5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 10.0*mult_wp;
  waypoint.pose.position.y = 5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 10.0*mult_wp;
  waypoint.pose.position.y = 10.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 15.0*mult_wp;
  waypoint.pose.position.y = 10.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 15.0*mult_wp;
  waypoint.pose.position.y = 5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 20.0*mult_wp;
  waypoint.pose.position.y = 5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 20.0*mult_wp;
  waypoint.pose.position.y = 10.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 25.0*mult_wp;
  waypoint.pose.position.y = 10.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 25.0*mult_wp;
  waypoint.pose.position.y = 5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 30.0*mult_wp;
  waypoint.pose.position.y = 5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 30.0*mult_wp;
  waypoint.pose.position.y = 10.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
*/

/* Caracol Plano
  waypoint.pose.position.x = 10.0*mult_wp;
  waypoint.pose.position.y = 10.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 10.0*mult_wp;
  waypoint.pose.position.y = 5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 15.0*mult_wp;
  waypoint.pose.position.y = 5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 15.0*mult_wp;
  waypoint.pose.position.y = 15.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 5.0*mult_wp;
  waypoint.pose.position.y = 15.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 5.0*mult_wp;
  waypoint.pose.position.y = 0.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 20.0*mult_wp;
  waypoint.pose.position.y = 0.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 20.0*mult_wp;
  waypoint.pose.position.y = 20.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 0.0*mult_wp;
  waypoint.pose.position.y = 20.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 0.0*mult_wp;
  waypoint.pose.position.y = -5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 25.0*mult_wp;
  waypoint.pose.position.y = -5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
*/

/* OCHO
  waypoint.pose.position.x = 7.5*mult_wp;
  waypoint.pose.position.y = 8.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 5.0*mult_wp;
  waypoint.pose.position.y = 11.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 3.0*mult_wp;
  waypoint.pose.position.y = 8.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 5.0*mult_wp;
  waypoint.pose.position.y = 5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 7.5*mult_wp;
  waypoint.pose.position.y = 8.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 10.0*mult_wp;
  waypoint.pose.position.y = 11.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 12.0*mult_wp;
  waypoint.pose.position.y = 8.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 10.0*mult_wp;
  waypoint.pose.position.y = 5.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 7.5*mult_wp;
  waypoint.pose.position.y = 8.0*mult_wp;
  waypoint.pose.position.z = 10.0*mult_wp;
  waypointList.push_back(waypoint);
*/

/* Base
  waypoint.pose.position.x = 5.0 * mult_wp;
  waypoint.pose.position.y = 5.0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 5.0 * mult_wp;
  waypoint.pose.position.y = 10.0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 5.0 * mult_wp;
  waypoint.pose.position.y = 10.0 * mult_wp;
  waypoint.pose.position.z = 5.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 5.0 * mult_wp;
  waypoint.pose.position.y = 5.0 * mult_wp;
  waypoint.pose.position.z = 5.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 10.0 * mult_wp;
  waypoint.pose.position.y = 5.0 * mult_wp;
  waypoint.pose.position.z = 5.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 10.0 * mult_wp;
  waypoint.pose.position.y = 5.0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 10.0 * mult_wp;
  waypoint.pose.position.y = 10.0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 15.0 * mult_wp;
  waypoint.pose.position.y = 10.0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 15.0 * mult_wp;
  waypoint.pose.position.y = 10.0 * mult_wp;
  waypoint.pose.position.z = 5.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 15.0 * mult_wp;
  waypoint.pose.position.y = 5.0 * mult_wp;
  waypoint.pose.position.z = 5.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 20.0 * mult_wp;
  waypoint.pose.position.y = 5.0 * mult_wp;
  waypoint.pose.position.z = 5.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 20.0 * mult_wp;
  waypoint.pose.position.y = 5.0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 20.0 * mult_wp;
  waypoint.pose.position.y = 10.0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 25.0 * mult_wp;
  waypoint.pose.position.y = 10.0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
*/

/* UNIA Zizag Pico
  waypoint.pose.position.x = 5.0 * mult_wp;
  waypoint.pose.position.y = 5.0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 7.5 * mult_wp;
  waypoint.pose.position.y = 2.5 * mult_wp;
  waypoint.pose.position.z = 10 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 10.0 * mult_wp;
  waypoint.pose.position.y = 0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 12.5 * mult_wp;
  waypoint.pose.position.y = 2.5 * mult_wp;
  waypoint.pose.position.z = 10 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 15.0 * mult_wp;
  waypoint.pose.position.y = 5 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 17.5 * mult_wp;
  waypoint.pose.position.y = 2.5 * mult_wp;
  waypoint.pose.position.z = 10 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 20.0 * mult_wp;
  waypoint.pose.position.y = 0 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 22.5 * mult_wp;
  waypoint.pose.position.y = 2.5 * mult_wp;
  waypoint.pose.position.z = 10 * mult_wp;
  waypointList.push_back(waypoint);
  waypoint.pose.position.x = 25.0 * mult_wp;
  waypoint.pose.position.y = 5 * mult_wp;
  waypoint.pose.position.z = 10.0 * mult_wp;
  waypointList.push_back(waypoint);
*/