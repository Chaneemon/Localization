#include <ros/ros.h>
#include <vector>
#include <cmath>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <kkanbu_msgs/C_Obstacle.h>
#include <kkanbu_msgs/C_Obstacles.h>

#define COLOR_IMAGE_WIDTH 848
#define COLOR_IMAGE_HEIGHT 480
#define DEPTH_IMAGE_WIDTH 848
#define DEPTH_IMAGE_HEIGHT 480
#define degree_to_radian (M_PI/180)

using namespace std;

class camera{
    private:
        float FOV = 87.;
        float Resolution = 848.;


    private:
        cv::Mat depth_image;

        ros::NodeHandle nh;

        ros::Subscriber image_depth; // realsense depth image
        ros::Subscriber yolo_bounding; // yolo bounding boxes

        ros::Publisher c_obstacle; // final camera obstacles


        void depthCb(const sensor_msgs::ImageConstPtr& msg);
        void boundingCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
        void imageCb(const sensor_msgs::ImageConstPtr& msg);

        kkanbu_msgs::C_Obstacle object_info(int xmin, int xmax, double depth, string name);
        kkanbu_msgs::C_Obstacle detect_obstacle(darknet_ros_msgs::BoundingBox bounding_box);
        double cal_depth(cv::Size size, cv::Point center);

    public:
        camera();
        ~camera();

};
