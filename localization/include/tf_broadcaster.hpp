#ifndef __TFBROADCASTER_HPP__
#define __TFBROADCASTER_HPP__

#include <string>
#include <deque>
#include <iostream>
#include <cmath>

// ROS header
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include "geometry_msgs/PoseStamped.h"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

class tfBroadcaster
{
    public:
        explicit tfBroadcaster();
        ~tfBroadcaster();

        double m_dMapHeight;
        const double Geod_a = 6378137.0;//SemiMajorAxis
        const double Geod_e2 = 0.00669437999014; // FirstEccentricitySquard, e ^ 2
        const double RAD2DEG = 180 / M_PI;
        const double DEG2RAD = M_PI / 180;
    
    private:
        ros::NodeHandle nh;
        ros::Subscriber m_sub_gps_groundtruth;
        ros::Subscriber m_sub_gps;
        ros::Subscriber m_sub_dr;
        ros::Subscriber m_sub_estimated_pose;

        void CallBackGPS(const geometry_msgs::PoseStamped::ConstPtr &msg);
        // void CallBackGPSGT(const geometry_msgs::PoseStamped::ConstPtr &msg);
        // void CallBackDR(const geometry_msgs::PoseStamped::ConstPtr &msg);
        //void CallBackNovatelINSPVAX(const novatel_oem7_msgs::INSPVAX::ConstPtr &msg);
        void CallBackEstimatedPose(const geometry_msgs::PoseStamped::ConstPtr &msg);

        geometry_msgs::PoseStamped ConvertToMapFrame(float lat, float lon, float hgt);
        double FnKappaLat(double dRef_Latitude, double dHeight);
        double FnKappaLon(double dRef_Latitude, double dHeight);
};

#endif
