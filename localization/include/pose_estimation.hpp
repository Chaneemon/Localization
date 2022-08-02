#ifndef __TEST__
#define __TEST__

#include <string>
#include <deque>
#include <iostream>
#include <cmath>

// ROS header
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Utility header

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

// Message header
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "kusv_msgs/State.h"
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/NavSatFix.h"
#include "ublox_msgs/NavPVT.h"
#include "sensor_msgs/Imu.h"
#include "ublox_gps/utils.h"
#include "mkgmtime.h"



#define _USE_MATH_DEFINES
#define _DEBUG_MODE (true)
#define _DISP_INPUT (false)

static const double Geod_a = 6378137.0;//SemiMajorAxis
static const double Geod_e2 = 0.00669437999014; // FirstEccentricitySquard, e ^ 2
static const double RAD2DEG = 180 / M_PI;
static const double DEG2RAD = M_PI / 180;

class poseEstimation
{
    struct GNSS{
        
        double sec;
        double nsec;
        double timestamp;
        double latitude;
        double longitude;
        double height;
        double heading;
        double headAcc;
        int gnssFlag;
        double pDOP;
        uint numSV;
        double hAcc;
        
    };

    struct VEHICLE{

        double sec;
        double nsec;
        double timestamp;
        double acc_mss;
        double velocity_ms;

        double roll_rad;
        double pitch_rad;
        double yaw_rad;
        
        double roll_rate_rads;
        double pitch_rate_rads;
        double yaw_rate_rads;

    };


    private:
        //double lat = 37.540032; //konkuk
        //double lon = 127.0709544;
        double lat = 37.2385024; //k-city
        double lon = 126.7708849;

        // // Gps Data
        double heading_deg = 0.0;
        // double heading_rad = 0.0;

        // IMU Raw Data
        double angular_velocity_x = 0.0;   // [rad/s]
        double angular_velocity_y = 0.0;   // [rad/s]
        double angular_velocity_z = 0.0;   // [rad/s]

        double linear_acceleration_x = 0.0;   // [m/s^2]
        double linear_acceleration_y = 0.0;   // [m/s^2]
        double linear_acceleration_z = 0.0;   // [m/s^2] 

        bool isInitGNSS;
        bool isInitLLH;
        bool gnssExistFlag;
        bool vehExistFlag;
        bool imuExistFlag;
        bool isFirstPrediction;
        bool isOnlyPrediction;
        bool isOnlyGNSS;

        geometry_msgs::PoseStamped init_gnss_enu;
        geometry_msgs::PoseStamped prev_pose;
        geometry_msgs::PoseStamped gnss_enu;


    public:
        explicit poseEstimation();
        virtual ~poseEstimation();
        void Run();
        void Prediction(double state[] ,VEHICLE inputIMU);
        void MeasurementUpdate(double state[], double covariance[3][3], GNSS inputGNSS, VEHICLE inputIMU);

        
    private:
        ros::NodeHandle nh;
        ros::Subscriber GNSS_sub;
        ros::Subscriber IMU_sub;
        ros::Subscriber VEH_sub;

        ros::Publisher m_pub_gnss_enu_pose;
        ros::Publisher m_pub_dead_reckoning_pose;
        ros::Publisher m_pub_estimated_pose;

        GNSS init_gnss;
        GNSS gnss;
        VEHICLE veh;
        
        const static int MEAS_ORDER = 3;
        const static int STATE_ORDER = 3;
        int cnt = 0;

        double egoVehicleState[3];
        double egoVehicleDR[3];
        double egoVehicleStateGT[3];
        double P_post[3][3] = 
        {
            1000., 0., 0.,
            0., 1000., 0.,
            0., 0., 1000.
        };
        
        double Q[3][3] = 
        {
            0.05, 0., 0.,
            0., 0.05, 0., 
            0., 0., 0.01
        };
        
        double R[3][3] = 
        {
            0.01, 0., 0.,
            0., 0.01, 0.,
            0., 0., 0.01
        };

    private:
      
        geometry_msgs::PoseStamped convert_to_map_frame(double lat, double lon, double hgt);
        sensor_msgs::NavSatFix convert_to_llh_frame(float east, float north, float hgt);

        double FnKappaLat(double dRef_Latitude, double dHeight);
        double FnKappaLon(double dRef_Latitude, double dHeight);
        
        void GNSS_Callback(const ublox_msgs::NavPVT::ConstPtr &msg);
        void IMU_Callback(const sensor_msgs::Imu::ConstPtr &msg);
        void VEH_Callback(const kusv_msgs::State::ConstPtr &msg);

};

#endif
