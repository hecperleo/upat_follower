#include <uav_abstraction_layer/ual.h>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"
#include <algorithm>
#include <fstream>
#include "ecl/containers.hpp"

#include "visualization_msgs/Marker.h"

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
grvc::ual::Velocity Vel_goClose;
// Visualizar look ahead y actual path
nav_msgs::Path pathDistToTarget, pathNormalDist, pathLookAhead, msgDrawActual;
std::vector<grvc::ual::Waypoint> vecAux;
// Vector de tiempos
std::vector<double> newVectorT;
// Subscripcion
void UALPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void UALVelocityCallback(const geometry_msgs::TwistStamped &msg);
void UALPathCallback(const nav_msgs::Path &msg);
void UALPathVCallback(const nav_msgs::Path &msg);
void newVectorTCallback(const nav_msgs::Path &msg);
// Mis funciones
void pure_pursuit();
void trayectoriaActual();
void funcCambiaLookAhead(int p);
void funcCambiaLookAheadVariable();
void funcDrawTriangles(int path_pos);
float funcMod(float x1, float x2, float y1, float y2, float z1, float z2);
float funcModDirection(float x1, float y1, float z1, float LA);
float funcCalcDistNormal();
float funcCalcDistNormalPos();
float funcSumDistNormal();
float movingAverageVx(float Vx);

visualization_msgs::Marker marker;

// Moving Average
using ecl::FiFo;
class movingAvg
{
public:
  movingAvg(const unsigned int widowsSize) : sum(0.0), average(0.0), windows_size(widowsSize), first(true)
  {
    fifo.resize(windows_size);
  }
  void reset()
  {
    fifo.fill(0);
    sum = 0;
    average = 0.0;
  }
  void update(const double &incomingData)
  {
    if (first)
    {
      fifo.fill(incomingData);
      first = false;
    }

    sum -= fifo[0];
    sum += incomingData;
    fifo.push_back(incomingData);
    average = sum / (double)(windows_size);
  }
  double sum;
  double average;
  unsigned int windows_size;
  FiFo<double> fifo;
  bool first;
};

// -----------------------------------------------------------------------------------------------------------

int main(int _argc, char **_argv)
{

  // grvc::utils::ArgumentParser args(_argc, _argv);
  // // Sirve para poner los servicios de UAL.
  // args.setArgument("ual_server", "on");
  ros::init(_argc, _argv, "pFollower");
  grvc::ual::UAL ual /* (args) */;

  while (!ual.isReady() && ros::ok())
  {
    std::cout << "[ TEST] UAL not ready!" << std::endl;
    sleep(1);
  }

  ros::NodeHandle nh;

  ros::Subscriber subPose =
      nh.subscribe("/uav_1/ual/pose", 0, UALPoseCallback);
  ros::Subscriber subActualVel =
      nh.subscribe("/uav_1/mavros/local_position/velocity", 0, UALVelocityCallback);
  ros::Subscriber subVel =
      nh.subscribe("velSpline", 0, UALPathVCallback);
  ros::Subscriber subPath =
      nh.subscribe("posSpline", 0, UALPathCallback);
  ros::Subscriber subNewVectorT =
      nh.subscribe("newVectorT", 0, newVectorTCallback);
  ros::Publisher pubToTarget =
      nh.advertise<nav_msgs::Path>("distToTarget", 1000);
  ros::Publisher pubNormalDist =
      nh.advertise<nav_msgs::Path>("distNormal", 1000);
  ros::Publisher pubLookAhead =
      nh.advertise<nav_msgs::Path>("distLookAhead", 1000);
  ros::Publisher pubDrawPathActual =
      nh.advertise<nav_msgs::Path>("drawActualPath", 1000);
  ros::Publisher vis_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  // Despega el dron
  ual.takeOff(flight_level, true);
  // Establece dos waypoints, uno como home y otro como punto inicial de la mision
  // (se usará en caso de que se desee)
  origen.header.frame_id = "map";
  origen.pose.position.x = 10.0;
  origen.pose.position.y = 0.0;
  origen.pose.position.z = flight_level;
  home.header.frame_id = "map";
  home.pose.position.x = 0.0;
  home.pose.position.y = 0.0;
  home.pose.position.z = flight_level;
  path_ok[0].pose.orientation.x = 0;
  path_ok[0].pose.orientation.y = 0;
  path_ok[0].pose.orientation.z = 0;
  path_ok[0].pose.orientation.w = 1;

  ual.goToWaypoint(path_ok[0], true);
  int windowsAvg = 3;
  movingAvg mavgVx(windowsAvg);
  movingAvg mavgVy(windowsAvg);
  movingAvg mavgVz(windowsAvg);
  for (int i = 0; i < windowsAvg; i++)
  {
    mavgVx.update(0);
    mavgVy.update(0);
    mavgVz.update(0);
  }
  ros::Duration(5).sleep();
  // Empieza a contar el tiempo para saber cuanto tarda en realizar la misión.
  ros::Time begin = ros::Time::now();
  // ---------------------------------------------- MISION ----------------------------------------------
  std::cout << "[ TEST] Start mission!" << '\n';
  float targetX, targetY, targetZ;
  float cont = 0;
  // Abre el fichero .dat
  std::ofstream fileVelocity, filePosition, fileMovingAvg, fileActualLA, fileT;
  fileVelocity.open("/home/hector/Matlab_ws/velocity.dat");
  filePosition.open("/home/hector/Matlab_ws/position.dat");
  fileMovingAvg.open("/home/hector/Matlab_ws/movingAvg.dat");
  fileActualLA.open("/home/hector/Matlab_ws/actualLA.dat");
  fileT.open("/home/hector/Matlab_ws/t.dat");
  // Moving average
  for (int p = 0; p < path_ok.size(); p++)
  {
    // Determina el look ahead de este instante en funcion del tiempo que tiene
    // el dron para llegar al waypoint objetivo.
    funcCambiaLookAhead(p); // Comentar para tener el generador V1
                            // Determina la distancia normal
    dist_normal = funcCalcDistNormal();
    // std::cout << "comienza pure pursuit" << '\n';
    // std::cout << "p = " << p << '\n';
    // Determina cual es el waypoint objetivo.
    pure_pursuit();
    p = p + pos_pure_pursuit;
    // std::cout << "p = " << p << '\n';
    // Asignación de variables del waypoint objetivo.
    targetX = path_ok[p].pose.position.x;
    targetY = path_ok[p].pose.position.y;
    targetZ = path_ok[p].pose.position.z;
    dist_toTarget = funcMod(targetX, actualPosX, targetY, actualPosY, targetZ, actualPosZ);
    // Publica el triangulos que se dibujan en rviz, donde los lados son: la distancia normal,
    // la distancia look ahead y la distancia hasta el objetivo.
    funcDrawTriangles(p);
    ros::spinOnce();
    // El dron está dentro del path. Se le otorga la velocidad necesaria para que vaya hacia
    // el waypoint objetivo.
    // std::cout << dist_normal << " < " << lookAhead << " & " << dist_toTarget << " > " << lookAhead << '\n';
    while (dist_normal < lookAhead && dist_toTarget > lookAhead)
    {
      dist_normal = funcCalcDistNormal();
      dist_toTarget = funcMod(targetX, actualPosX, targetY, actualPosY, targetZ, actualPosZ);
      mavgVx.update(targetX - actualPosX);
      mavgVy.update(targetY - actualPosY);
      mavgVz.update(targetZ - actualPosZ);
      Vel_goClose.twist.linear.x = mavgVx.average;
      Vel_goClose.twist.linear.y = mavgVy.average;
      Vel_goClose.twist.linear.z = mavgVz.average;
      ual.setVelocity(Vel_goClose);
      fileMovingAvg << mavgVx.average << " "
                    << mavgVy.average << " "
                    << mavgVz.average << "\n";

      // Escribe en un .dat
      fileVelocity << Vel_goClose.twist.linear.x << " " << actualVelX << " "
                   << Vel_goClose.twist.linear.y << " " << actualVelY << " "
                   << Vel_goClose.twist.linear.z << " " << actualVelZ << "\n";
      filePosition << targetX << " " << actualPosX << " "
                   << targetY << " " << actualPosY << " "
                   << targetZ << " " << actualPosZ << "\n";
      fileActualLA << lookAhead << " " << newVectorT[p] << "\n";

      // Publica
      pubNormalDist.publish(pathNormalDist);
      pubToTarget.publish(pathDistToTarget);
      pubLookAhead.publish(pathLookAhead);
      vis_pub.publish(marker);
      trayectoriaActual();
      pubDrawPathActual.publish(msgDrawActual);
      i++;
      // Espera
      ros::Duration(0.1).sleep();
      if (!(dist_normal < lookAhead && dist_toTarget > lookAhead))
      {
        continue;
      }
    }
    // El dron está fuera del path. Se le otorga la velocidad necesaria para que vaya hacia
    // el waypoint objetivo.
    while (dist_normal > lookAhead)
    {
      dist_normal = funcCalcDistNormal();
      mavgVx.update(targetX - actualPosX);
      mavgVy.update(targetY - actualPosY);
      mavgVz.update(targetZ - actualPosZ);
      Vel_goClose.twist.linear.x = mavgVx.average;
      Vel_goClose.twist.linear.y = mavgVy.average;
      Vel_goClose.twist.linear.z = mavgVz.average;
      ual.setVelocity(Vel_goClose);
      fileMovingAvg << mavgVx.average << " "
                    << mavgVy.average << " "
                    << mavgVz.average << "\n";

      // Escribe en un .dat
      fileVelocity << Vel_goClose.twist.linear.x << " " << actualVelX << " "
                   << Vel_goClose.twist.linear.y << " " << actualVelY << " "
                   << Vel_goClose.twist.linear.z << " " << actualVelZ << "\n";
      filePosition << targetX << " " << actualPosX << " "
                   << targetY << " " << actualPosY << " "
                   << targetZ << " " << actualPosZ << "\n";
      fileActualLA << lookAhead << " " << newVectorT[p] << "\n";

      // Publica
      trayectoriaActual();
      pubDrawPathActual.publish(msgDrawActual);
      vis_pub.publish(marker);
      i++;

      // Espera
      ros::Duration(0.1).sleep();
      if (!(dist_normal > lookAhead))
      {
        continue;
      }
    }
    // Calcula la suma de todas las distancias normales.
    funcSumDistNormal();
    // Cuenta los cambios de waypoint.
    cont++;
  }
  // Escribe cuanto tiempo se ha tardado en hacer la mision en un .dat
  fileT << ros::Time::now() - begin << std::endl;
  // Cierra el .dat
  fileVelocity.close();
  filePosition.close();
  fileMovingAvg.close();
  fileActualLA.close();
  fileT.close();
  // ---------------------------------------------- MISION ----------------------------------------------

  std::cout << "[ TEST] Mission complete! " << std::endl;
  std::cout << "[ TEST] Time: " << ros::Time::now() - begin << std::endl;
  std::cout << "[ TEST] Media dist_normal: " << funcSumDistNormal() / cont << '\n';
  /*std::cout << "[ TEST] Going home: [" << home.pose.position.x << "," << \
                home.pose.position.y << "," << home.pose.position.z << \
                "] - frame_id: " << home.header.frame_id << std::endl;
    ual.goToWaypoint(home, true);
    std::cout << "[ TEST] Arrived!" << std::endl;*/
  std::cout << "[ TEST] -----------------------------------" << std::endl;

  ual.land(1);

  return 0;
}

// -----------------------------------------------------------------------------------------------------------

void pure_pursuit()
{
  float comp_dist_resta;
  std::vector<float> comp_dist;
  std::vector<float>::iterator up;
  // Obtiene la magnitud de la distancia normal.
  dist_normal = funcCalcDistNormal();
  // Obtiene la posición en el path de la distancia normal.
  pos_path = funcCalcDistNormalPos();
  // std::cout << "pos path = " << pos_path << '\n';
  // Determinar que waypoint está a una distancia similar a la distancia look ahead,
  // respecto del waypoint que corresponde con la normal del dron en este instante.
  if (dist_normal < lookAhead)
  {
    // std::cout << "dist_normal mas pequeña que look ahead" << '\n';
    for (pos_path; pos_path < path_ok.size() - 1; pos_path++)
    {
      comp_dist_resta = funcModDirection(path_ok[pos_path].pose.position.x,
                                         path_ok[pos_path].pose.position.y,
                                         path_ok[pos_path].pose.position.z, lookAhead);

      comp_dist.push_back(comp_dist_resta);
    }
    up = std::upper_bound(comp_dist.begin(), comp_dist.end(), lookAhead);
    pos_pure_pursuit = up - comp_dist.begin();
    // std::cout << "pos pure pursuit = " << pos_pure_pursuit << '\n';
    // Se actualiza la distancia look ahead en funcion de como evoluciona la distancia
    // normal en el timepo.
    // std::cout << "cambia LookAhead" << '\n';
    // funcCambiaLookAheadVariable(); // Descomentar para tener el generador V1
    // Si el dron está lejos del path, se le da la orden de que se aproxime por la
    // distancia mas corta.
  }
  else if (dist_normal > lookAhead)
  {
    pos_pure_pursuit = pos_path;
    flagPurePursuit = false;
    std::cout << "[ TEST] Going Closer" << '\n';
  }
  // Limpia el vector
  comp_dist.clear();
}

void funcCambiaLookAhead(int p)
{
  // Si el lookAhead del estado anterior es diferente que el waypoint objetivo,
  // suma o resta un porcentaje para suavizar las aceleraciones.
  /*if 			 	((1/newVectorT[p]) !=  prev_lookAhead){
		if      ((1/newVectorT[p]) > prev_lookAhead){
			lookAhead = prev_lookAhead + 1 * 0.001;
		}else if((1/newVectorT[p]) < prev_lookAhead){
			lookAhead = prev_lookAhead - 1 * 0.001;
		}
	}else if  ((1/newVectorT[p]) == prev_lookAhead){
		lookAhead = 1/newVectorT[p];
	}
	prev_lookAhead = lookAhead;*/
  float targetX = path_ok[p].pose.position.x;
  float targetY = path_ok[p].pose.position.y;
  float targetZ = path_ok[p].pose.position.z;
  dist_toTarget = funcMod(targetX, actualPosX, targetY, actualPosY, targetZ, actualPosZ);
  lookAhead = 1 / newVectorT[p];
  std::cout << "[ TEST] Look Ahead = " << lookAhead << " | Target = " << 1 / newVectorT[p] << " | P = " << p << '\n';
}

void funcCambiaLookAheadVariable()
{
  dist_normal = funcCalcDistNormal();
  // Si el dron está lejos del path, la distancia look ahead toma el valor de la distancia
  // normal en ese momento.
  // std::cout << "dist normal      = " << dist_normal << '\n';
  // std::cout << "dist normal prev = " << dist_normal_prev << '\n';
  // std::cout << "lookAhead        = " << lookAhead << '\n';
  if (dist_normal > lookAhead_init * 3)
  {
    lookAhead = lookAhead - lookAhead_init * 0.1;
    // std::cout << "[ TEST] Lejos -> dist_normal = " << dist_normal << ", lookAhead = " << lookAhead << '\n';
    // Si la distancia normal disminuye respecto del instante anterior y el dron está cerca del path
    // aumenta la distancia look ahead un 10% de la distancia look ahead inicial.
  }
  else if (dist_normal < dist_normal_prev * 0.95 && lookAhead < lookAhead_init * 3)
  {
    // std::cout << "[ TEST] Suma  -> dist_normal = " << dist_normal << ", lookAhead = " << lookAhead << '\n';
    lookAhead = lookAhead + lookAhead_init * 0.1;
    // Si la distancia normal aumenta respecto del instante anterior y el dron está cerca del path
    // disminuye la distancia look ahead un 10% de la distancia look ahead inicial.
  }
  else if (dist_normal > dist_normal_prev * 1.05 && lookAhead > lookAhead_init * 1.05)
  {
    lookAhead = lookAhead - lookAhead_init * 0.1;
    // std::cout << "[ TEST] Resta -> dist_normal = " << dist_normal << ", lookAhead = " << lookAhead << '\n';
  }
  // Guarda el valor actual de la distancia normal, para poder utilizarlo en el siguiente instante
  // como comprobación.
  dist_normal_prev = dist_normal;
}

float funcMod(float x1, float x2, float y1, float y2, float z1, float z2)
{
  float mod;
  mod = sqrt((x2 - x1) * (x2 - x1) +
             (y2 - y1) * (y2 - y1) +
             (z2 - z1) * (z2 - z1));
  return mod;
}

float funcModDirection(float x1, float y1, float z1, float LA)
{
  float mod;
  float x2, y2, z2;
  // Calcula el módulo teniendo en cuenta el signo del waypoint para poder sumar
  // la distancia look ahead correctamente.
  if (x1 < 0)
  {
    x2 = x1 - LA;
  }
  if (x1 > 0)
  {
    x2 = x1 + LA;
  }
  if (y1 < 0)
  {
    y2 = y1 - LA;
  }
  if (y1 > 0)
  {
    y2 = y1 + LA;
  }
  if (z1 < 0)
  {
    z2 = z1 - LA;
  }
  if (z1 > 0)
  {
    z2 = z1 + LA;
  }

  mod = funcMod(x1, x2, y1, y2, z1, z2);

  return mod;
}

float funcCalcDistNormal()
{
  std::vector<float> smallest_dist;
  float smallest_dist_sqrt;
  for (int x = 0; x < path_ok.size() - 1; x++)
  {
    smallest_dist_sqrt = funcMod(path_ok[x].pose.position.x, actualPosX,
                                 path_ok[x].pose.position.y, actualPosY,
                                 path_ok[x].pose.position.z, actualPosZ);
    smallest_dist.push_back(smallest_dist_sqrt);
  }
  auto smallest_dist_min = std::min_element(smallest_dist.begin(), smallest_dist.end());
  // Retorna la magnitud de la distancia normal
  return *smallest_dist_min;
}

float funcCalcDistNormalPos()
{
  std::vector<float> smallest_dist;
  float smallest_dist_sqrt;
  for (int x = 0; x < path_ok.size() - 1; x++)
  {
    smallest_dist_sqrt = funcMod(path_ok[x].pose.position.x, actualPosX,
                                 path_ok[x].pose.position.y, actualPosY,
                                 path_ok[x].pose.position.z, actualPosZ);
    smallest_dist.push_back(smallest_dist_sqrt);
  }
  auto smallest_dist_min = std::min_element(smallest_dist.begin(), smallest_dist.end());
  auto comp_dist_pos = smallest_dist_min - smallest_dist.begin();
  // Retorna la posición en el path de la distancia normal
  return comp_dist_pos;
}

float funcSumDistNormal()
{
  suma_distNormal = suma_distNormal + funcCalcDistNormal();
  return suma_distNormal;
}

void funcDrawTriangles(int p)
{
  // visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = actualPosX;
  marker.pose.position.y = actualPosY;
  marker.pose.position.z = actualPosZ - 0.2;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.8;
  marker.scale.y = 0.8;
  marker.scale.z = 0.6;
  marker.color.a = 0.2; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  // --------------------
  std::vector<grvc::ual::Waypoint> wpList_distToTarget;
  std::vector<grvc::ual::Waypoint> wpList_distNormal;
  std::vector<grvc::ual::Waypoint> wpList_lookAhead;
  grvc::ual::Waypoint wp_distToTarget;
  grvc::ual::Waypoint wp_distNormal;
  grvc::ual::Waypoint wp_lookAhead;

  wp_distToTarget.pose.position.x = actualPosX;
  wp_distToTarget.pose.position.y = actualPosY;
  wp_distToTarget.pose.position.z = actualPosZ;
  wpList_distToTarget.push_back(wp_distToTarget);
  wp_distToTarget.pose.position.x = path_ok[p].pose.position.x;
  wp_distToTarget.pose.position.y = path_ok[p].pose.position.y;
  wp_distToTarget.pose.position.z = path_ok[p].pose.position.z;
  wpList_distToTarget.push_back(wp_distToTarget);
  std::vector<geometry_msgs::PoseStamped> posesToTarget(wpList_distToTarget.size());
  for (int x = 0; x < wpList_distToTarget.size(); x++)
  {
    posesToTarget.at(x).pose.position.x = wpList_distToTarget[x].pose.position.x;
    posesToTarget.at(x).pose.position.y = wpList_distToTarget[x].pose.position.y;
    posesToTarget.at(x).pose.position.z = wpList_distToTarget[x].pose.position.z;
  }
  pathDistToTarget.header.frame_id = "map";
  pathDistToTarget.poses = posesToTarget;

  pos_path = funcCalcDistNormalPos();

  wp_distNormal.pose.position.x = actualPosX;
  wp_distNormal.pose.position.y = actualPosY;
  wp_distNormal.pose.position.z = actualPosZ;
  wpList_distNormal.push_back(wp_distNormal);
  wp_distNormal.pose.position.x = path_ok[pos_path].pose.position.x;
  wp_distNormal.pose.position.y = path_ok[pos_path].pose.position.y;
  wp_distNormal.pose.position.z = path_ok[pos_path].pose.position.z;
  wpList_distNormal.push_back(wp_distNormal);
  std::vector<geometry_msgs::PoseStamped> posesNormal(wpList_distNormal.size());
  for (int x = 0; x < wpList_distNormal.size(); x++)
  {
    posesNormal.at(x).pose.position.x = wpList_distNormal[x].pose.position.x;
    posesNormal.at(x).pose.position.y = wpList_distNormal[x].pose.position.y;
    posesNormal.at(x).pose.position.z = wpList_distNormal[x].pose.position.z;
  }
  pathNormalDist.header.frame_id = "map";
  pathNormalDist.poses = posesNormal;

  wp_lookAhead.pose.position.x = path_ok[p].pose.position.x;
  wp_lookAhead.pose.position.y = path_ok[p].pose.position.y;
  wp_lookAhead.pose.position.z = path_ok[p].pose.position.z;
  wpList_lookAhead.push_back(wp_lookAhead);
  wp_lookAhead.pose.position.x = path_ok[pos_path].pose.position.x;
  wp_lookAhead.pose.position.y = path_ok[pos_path].pose.position.y;
  wp_lookAhead.pose.position.z = path_ok[pos_path].pose.position.z;
  wpList_lookAhead.push_back(wp_lookAhead);
  std::vector<geometry_msgs::PoseStamped> posesLookAhead(wpList_lookAhead.size());
  for (int x = 0; x < wpList_lookAhead.size(); x++)
  {
    posesLookAhead.at(x).pose.position.x = wpList_lookAhead[x].pose.position.x;
    posesLookAhead.at(x).pose.position.y = wpList_lookAhead[x].pose.position.y;
    posesLookAhead.at(x).pose.position.z = wpList_lookAhead[x].pose.position.z;
  }
  pathLookAhead.header.frame_id = "map";
  pathLookAhead.poses = posesLookAhead;
}

void trayectoriaActual()
{
  grvc::ual::Waypoint waypoint;
  // Se guarda en un vector auxiliar la posicion de cada instante
  waypoint.pose.position.x = actualPosX;
  waypoint.pose.position.y = actualPosY;
  waypoint.pose.position.z = actualPosZ;
  vecAux.push_back(waypoint);
  // Se crea el vector de la trayectoria actual
  std::vector<geometry_msgs::PoseStamped> posesActual(i + 1);
  msgDrawActual.header.frame_id = "map";
  for (int j = 0; j < posesActual.size() - 1; j++)
  {
    // Se coloca en la trayectoria actual, las posiciones seguidas con anterioridad
    posesActual.at(j).pose.position.x = vecAux.at(j).pose.position.x;
    posesActual.at(j).pose.position.y = vecAux.at(j).pose.position.y;
    posesActual.at(j).pose.position.z = vecAux.at(j).pose.position.z;
  }
  // Se coloca en la trayectoria actual, la posicion actual
  posesActual.at(i).pose.position.x = actualPosX;
  posesActual.at(i).pose.position.y = actualPosY;
  posesActual.at(i).pose.position.z = actualPosZ;

  msgDrawActual.poses = posesActual;
}

void UALPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  actualPosX = msg->pose.position.x;
  actualPosY = msg->pose.position.y;
  actualPosZ = msg->pose.position.z;

  return;
}

void UALVelocityCallback(const geometry_msgs::TwistStamped &msg)
{
  actualVelX = msg.twist.linear.x;
  actualVelY = msg.twist.linear.y;
  actualVelZ = msg.twist.linear.z;

  return;
}

void UALPathCallback(const nav_msgs::Path &msg)
{
  grvc::ual::Waypoint waypoint;
  if (flagSubPath == true)
  {
    for (int p = 0; p < msg.poses.size(); p++)
    {
      waypoint.header.frame_id = msg.header.frame_id;
      waypoint.pose.position.x = msg.poses.at(p).pose.position.x;
      waypoint.pose.position.y = msg.poses.at(p).pose.position.y;
      waypoint.pose.position.z = msg.poses.at(p).pose.position.z;
      path_ok.push_back(waypoint);
    }
    // std::cout << "********************************** path ok size = " << path_ok.size() << '\n';
  }
  flagSubPath = false;

  return;
}

void UALPathVCallback(const nav_msgs::Path &msg)
{
  grvc::ual::Velocity v;
  if (flagSubVel == true)
  {
    for (int p = 0; p < msg.poses.size(); p++)
    {
      v.twist.linear.x = msg.poses.at(p).pose.position.x;
      v.twist.linear.y = msg.poses.at(p).pose.position.y;
      v.twist.linear.z = msg.poses.at(p).pose.position.z;
      path_v.push_back(v);
    }
  }
  flagSubVel = false;

  return;
}

void newVectorTCallback(const nav_msgs::Path &msg)
{
  if (flagSubVectorT == true)
  {
    for (int p = 0; p < msg.poses.size(); p++)
    {
      newVectorT.push_back(msg.poses.at(p).pose.position.x);
    }
  }
  flagSubVectorT = false;
  return;
}
