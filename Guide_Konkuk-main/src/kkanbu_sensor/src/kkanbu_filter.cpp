// #include "ros/ros.h"
// #include <kkanbu_msgs/Obstacles.h>
// #include <kkanbu_msgs/SegmentObstacle.h>
// #include <kkanbu_msgs/CircleObstacle.h>
// #include <kkanbu_msgs/SensorPointArray.h>
// #include <kkanbu_msgs/SensorPoint.h>
// #include <kkanbu_msgs/current_state.h>

// #define car_radius 0.4

// ros::Publisher final_pub;

// void chatterCallback(const kkanbu_msgs::Obstacles lidar_info)
// {
  
// }

// /*void IMU_Callback(const sensor_info::current_state cs)
// {
//   sensor_info::FinalObstacle msg;
//   msg.current_speed = cs.velocity;
//   msg.yaw = cs.yaw;
//   msg.yaw_rate = cs.yaw_rate;
  
// }*/

// //  ROS_INFO_STREAM("I heard"<< lidar_info.circles[0]);

// /*geometry_msgs/Point center      # Central point [m]
// geometry_msgs/Vector3 velocity  # Linear velocity [m/s]
// float64 radius                  # Radius with added margin [sssm]
// float64 true_radius             # True measured radius [m]
// float64 angle			# Angle of Center [degree]
// float64 c_distance		# Distance from Center [m]
// float64 min_distance
// */

// int main(int argc, char **argv)
// {
//    ros::init(argc, argv, "sensor");
//    ros::NodeHandle nh;
   
//    final_pub = nh.advertise<kkanbu_msgs::SensorPointArray>("/obs", 10);

//    ros::Subscriber sub_ob = nh.subscribe("raw_obstacles",10,chatterCallback);
//    //ros::Subscriber sub_cs = nh.subscribe("/current_state",10,IMU_Callback);
//    ros::spin();

//    return 0;
// }

