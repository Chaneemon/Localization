#include <kkanbu_camera.hpp>

camera::camera(){
  image_depth = nh.subscribe("/camera/depth/image_rect_raw", 1, &camera::depthCb, this);
  yolo_bounding = nh.subscribe("/darknet_ros/bounding_boxes", 1, &camera::boundingCb, this);
  image_ = nh.subscribe("/camera/color/image_raw", 1, &camera::cameraCb, this);


  c_obstacle = nh.advertise<kkanbu_msgs::C_Obstacles>("C_Obstacles", 1); 

  depth_image = cv::Mat(cv::Size(DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT),CV_16UC1);
}

camera::~camera(){}


double camera::cal_depth(cv::Size size, cv::Point center){
  int radius;
  vector<cv::Point> pix_index;

  if(size.width <= size.height){
    radius = int(size.width/5);
  }
  else{
    radius = int(size.height/5);
  }

  cv::Mat mask = cv::Mat::zeros(cv::Size(COLOR_IMAGE_WIDTH, COLOR_IMAGE_HEIGHT), CV_8UC1);
  cv::circle(mask, center, radius, cv::Scalar(255),-1);
  
  for (int i = 0; i < mask.rows; ++i){
    for (int j = 0; j < mask.cols; ++j){
      if(mask.at<uchar>(i,j) == 255){
        pix_index.push_back(cv::Point(j,i));
      }
    }
  }

  double depth_sum = 0;
  int count = 0;

  for(int i = 0; i < pix_index.size(); i++){
    if(depth_image.at<ushort>(pix_index[i]) != 0){
      depth_sum += depth_image.at<ushort>(pix_index[i]);
      count += 1;
    }
  }

  if(count == 0){
    count = 1;
  }
  double avg_depth = depth_sum / count;

  return avg_depth;
}

kkanbu_msgs::C_Obstacle camera::object_info(int xmin, int xmax, double depth, string name)
{
  int c_x = (xmax + xmin)/2;
  double angle = (c_x - 424) / Resolution * FOV ;
  double angle_xmin = (xmin - 424) / Resolution * FOV * degree_to_radian ;
  double angle_xmax = (xmax - 424) / Resolution * FOV * degree_to_radian;
  double width = (tan(angle_xmax) - tan(angle_xmin)) * depth;

  kkanbu_msgs::C_Obstacle obstacle;
  
  obstacle.angle = angle;
  obstacle.width = width;
  obstacle.distance = depth;
  obstacle.name = name;

  return obstacle;
}
void camera::cameraCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr1;
  try
  {
    cv_ptr1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {

    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // cout << "depth size!!"<<cv_ptr->image.size()<<endl;
  image= cv_ptr1->image;
  ROS_INFO("image col : %d", image.cols);
  ROS_INFO("image row : %d", image.rows);

}
void camera::depthCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // cout << "depth size!!"<<cv_ptr->image.size()<<endl;
  depth_image= cv_ptr->image;
  ROS_INFO("detph col : %d",  depth_image.cols);
  ROS_INFO("depth row : %d", depth_image.rows);

}

void camera::boundingCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  kkanbu_msgs::C_Obstacles c_obs;

  for(int i = 0; i < boost::size(msg->bounding_boxes); i++){
    if (msg->bounding_boxes[i].Class == "cup"){
      cout << "pass" << endl;
      continue;
    }
    kkanbu_msgs::C_Obstacle ob = detect_obstacle(msg->bounding_boxes[i]);
    if (ob.width != 0){
      c_obs.camera_obstacle.push_back(ob);
    }
  }
  if(image.cols == depth_image.cols && image.rows == depth_image.rows ){
    c_obstacle.publish(c_obs);
  }
  else{
    ROS_INFO("no match image size");
  }

}

kkanbu_msgs::C_Obstacle camera::detect_obstacle(darknet_ros_msgs::BoundingBox bounding_box){
  cv::Point center((bounding_box.xmin + bounding_box.xmax)/2, (bounding_box.ymin + bounding_box.ymax)/2);
  cv::Size bound_size((bounding_box.xmax - bounding_box.xmin),(bounding_box.ymax-bounding_box.ymin));

  double depth = cal_depth(bound_size, center)/1000; // m 
  // cout << depth << endl;

  kkanbu_msgs::C_Obstacle ob = object_info(bounding_box.xmin, bounding_box.xmax, depth, bounding_box.Class);

  return ob;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_detection");

  camera camera;

  ros::spin();

  return 0;
}

