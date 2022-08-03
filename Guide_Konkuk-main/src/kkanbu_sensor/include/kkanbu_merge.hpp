#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <point.h>

#include <kkanbu_msgs/C_Obstacle.h>
#include <kkanbu_msgs/C_Obstacles.h>
#include <kkanbu_msgs/Obstacles.h>
#include <kkanbu_msgs/SensorPoint.h>
#include <kkanbu_msgs/SensorPointArray.h>

using namespace sensor_fusion;

class merge_circle{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_lidar_circle;
        ros::Subscriber sub_camera_circle;
        ros::Publisher markerArrayPub;
        ros::Publisher camera_markerArrayPub;
        ros::Publisher merge_markerArrayPub;

        ros::Publisher final_markerArrayPub;
        ros::Publisher final_pub;

        geometry_msgs::PoseArray lidar_circle_array;
        geometry_msgs::PoseStamped wp;
        visualization_msgs::MarkerArray D_WPArray;
        kkanbu_msgs::C_Obstacles camera_obstacle;
        kkanbu_msgs::Obstacles lidar_circle;
        kkanbu_msgs::Obstacles camera_circle;
        kkanbu_msgs::Obstacles merge_circles;
        kkanbu_msgs::SensorPointArray final_circles;
        kkanbu_msgs::SensorPoint sp;

        int num = 0; 

    public:
        merge_circle();
        ~merge_circle();

        void lidarCallback(kkanbu_msgs::Obstacles _lidar_circle );
        void CameraCallback(kkanbu_msgs::C_Obstacles _camera_obstacle ); // to edit
        void viz_lidar_circle();
        void viz_camera_circle();
        void viz_merge_circle();
        void viz_final_circle();
        void merge();
        void filter();
        //visualization_msgs::Marker markergenerator(geometry_msgs::PoseStamped point,double scale,double scale_z, int _id, double color);
        visualization_msgs::Marker markergenerator(geometry_msgs::PoseStamped point,double scale,double scale_z, int _id, double r, double g, double b);
};

