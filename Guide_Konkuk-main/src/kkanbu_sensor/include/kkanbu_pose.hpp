#include <string>
#include <deque>
#include <iostream>

#define _USE_MATH_DEFINES 
#include <cmath>

// ROS header
#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
                                                                                                                                                                                                                                                                

// Message header
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"

#include "sensor_msgs/Imu.h"

#include "ublox_msgs/NavPVT.h"
#include "ublox_gps/utils.h"

#include <kkanbu_msgs/current_state.h>


static const double Geod_a = 6378137.0; // SemiMajorAxis
static const double Geod_e2 = 0.00669437999014; // FirstEccentricitySquard, e ^ 2
static const double RAD2DEG = 180 / M_PI;
static const double DEG2RAD = M_PI / 180;

class poseEstimation
{
    struct local{
        double x;
        double y;
    };
    
    struct current_state{
        double x;
        double y;
        double velocity;
        double yaw;
        double yaw_rate;
    };

    private:
        // double lat = 37.5424; // base
        // double lon = 127.0775;

        double m_dRefLatitude_deg = 37.5424; // base
        double m_dRefLongitude_deg = 127.0775;

        bool isInitGNSS;
        bool isInitIMU;


        local prev_local;
        local gnss_enu;

    public:
        explicit poseEstimation();
        virtual ~poseEstimation();
        void publish_currentstate();
        
    private:
        ros::NodeHandle nh;

        ros::Subscriber GNSS_sub;
        ros::Subscriber IMU_sub;

        ros::Publisher m_pub_current_state;

        local init_local;
        local local;

        current_state current_state;

        double prev_yaw;
        double imu_prev_time;
        double gnss_prev_time;
        
        double distance;
        double heading_deg;

    private:
        struct local convert_to_map_frame(double lat, double lon);

        double FnKappaLat(double dRef_Latitude, double dHeight);
        double FnKappaLon(double dRef_Latitude, double dHeight);
        
        void GNSS_Callback(const ublox_msgs::NavPVT::ConstPtr &msg);
        void IMU_Callback(const sensor_msgs::Imu:: ConstPtr &msg);
};
