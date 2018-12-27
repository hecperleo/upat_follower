#include <ros/ros.h>

#include <tf/tf.h>
#include <uav_abstraction_layer/ual.h>
#include <fstream>
#include "ecl/geometry.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
// Para el interp1
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

class Manager {
   public:
    Manager();
    ~Manager();

    void loop();

   private:
    // Callbacks
    // void matrix_cb(const mapping::vectorVector input);

    //
    void params();
    void UALPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg1);
    void UALPathCallback(const nav_msgs::Path &msg);
    void vectorTCallback(const nav_msgs::Path &msg);
    void spline();
    void eclSpline(float minT, float dist_total);
    void funcInterpvectorT(int splineSize);
    void funcPreprocesamiento();
    void funcCompruebaTiempos();
    float funcMod(float x1, float x2, float y1, float y2, float z1, float z2);
    bool funcOtraSpline(std::vector<double> vVz, int splineSize, bool safe);
    std::vector<double> funcAumentaVector(std::vector<double> vect, int finalSize);
    std::vector<double> interpolaWps(std::vector<double> wp, double t);
    template <typename Real>
    int nearestNeighbourIndex(std::vector<Real> &x, Real &value);
    template <typename Real>
    std::vector<Real> interp1(std::vector<Real> &x, std::vector<Real> &y, std::vector<Real> &x_new);
    nav_msgs::Path Construct_Path_Msg(double *x, double *y, double *z, int length);

    // Node handlers
    ros::NodeHandle n;

    // Subscribers
    ros::Subscriber subPose, subPath, subvectorT;

    // Publishers
    ros::Publisher pubDrawPath, pubEclPath, pubEclPathV, pubEclPath1, pubEclPath2, pubnewvectorT;

    // Variables
    float t;
    int i = 0;
    float sumVecT = 0;
    const float velocidadMax = 1;
    bool flagSpline = true;
    bool flagSubPath = true;
    bool flagSubvectorT = true;
    bool lastOne = false;
    bool flagFinishSpline = false;
    double actualPosX, actualPosY, actualPosZ;

    grvc::ual::Waypoint waypoint;
    std::vector<grvc::ual::Waypoint> vecAux;
    std::vector<double> poseListX, poseListY, poseListZ;
    std::vector<double> newPoseListX, newPoseListY, newPoseListZ;
    std::vector<double> vectorT, newvectorT, lastSplinevectorT;
    nav_msgs::Path msgDrawPath, msgnewvectorT, path, smoothedPath, splinePath, spline_msg, splineV_msg, spline_msg1, spline_msg2;

    // Params
};