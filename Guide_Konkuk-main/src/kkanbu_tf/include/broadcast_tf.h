#ifndef BROADCAST_TF_H
#define BROADCAST_TF_H

#include <math.h>
#include <ros/ros.h>
#include "kkanbu_msgs/ControlCommand.h"
#include "kkanbu_msgs/VehicleState.h"
#include "kkanbu_msgs/current_state.h"
#include <tf/tf.h>
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Pose.h"

#define PI 3.14159265358979323846 /* pi */

class TF_Broadcaster {
    private:
        ros::NodeHandle nh_;

        tf::TransformBroadcaster broadcast_TF;

        ros::Subscriber sub_VehicleState;

    public:
        TF_Broadcaster();
        ~TF_Broadcaster();
        
        void broadcast_all_tf();

        //tf
        void broadcasting_mapToEgo_tf();
        void broadcasting_egoToSensor_tf();
        
        //Callback func
        // void get_VehicleState(const kkanbu_msgs::VehicleState::ConstPtr& msg);
        void get_VehicleState(const kkanbu_msgs::current_state::ConstPtr& msg);
        
        // Func
        void Update_Ego_TF();
    private:
        // Input
        // kkanbu_msgs::VehicleState ego_state_;
        kkanbu_msgs::current_state ego_state_;
        
        //variables
        double ego_init_x_, ego_init_y_, ego_init_yaw_;
        double sensor1_pose_x_,sensor1_pose_y_,sensor1_pose_z_;
        bool isStateUpdate = false;

        tf::Quaternion map2ego_q_;
        tf::Transform map2ego_transform_;
};

#endif //BROADCAST_TF_H
