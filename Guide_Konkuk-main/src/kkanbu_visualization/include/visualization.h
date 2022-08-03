#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "kkanbu_msgs/VehicleState.h"
#include "kkanbu_msgs/ControlCommand.h"

#include <math.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include <ros/ros.h>
#include <tf/tf.h>

#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Image.h"

#define PI 3.14159265358979323846 /* pi */

class Visualizatoin{
    private:
        ros::NodeHandle nh_;

        ros::Publisher pub_vehicleModelMarker;
        ros::Publisher pub_vehicleSpeed;
        ros::Publisher pub_commandSteering;
        ros::Publisher pub_teamName;

        ros::Subscriber sub_vehicleState;
        ros::Subscriber sub_controlCommand;

        visualization_msgs::Marker vehicle_model_marker_;
        // Input
        kkanbu_msgs::VehicleState ego_state_;
        kkanbu_msgs::ControlCommand ego_cmd_;

        // Output
        sensor_msgs::Image controller;
        std_msgs::Float32 ego_velocity_;
        std_msgs::Float32 ego_steering_;
        std_msgs::String team_name_;

    public:
        Visualizatoin();
        ~Visualizatoin();
    public:
        // Callback
        void get_vehicleState(const kkanbu_msgs::VehicleState::ConstPtr& msg);
        void get_controlCommand(const kkanbu_msgs::ControlCommand::ConstPtr& msg);    
        void makeVehicleModelMarker();
        void publishAllMarker();

};



#endif