#include "broadcast_tf.h"

TF_Broadcaster::TF_Broadcaster(){
    // Get Parameter from launch
    nh_.param("kkanbu_tf/ego/init_x", ego_init_x_, 0.0);
    nh_.param("kkanbu_tf/ego/init_y", ego_init_y_, 0.0);
    nh_.param("kkanbu_tf/ego/init_yaw", ego_init_yaw_, 0.0);
    nh_.param("kkanbu_tf/sensor1/pose_x", sensor1_pose_x_, 0.0);
    nh_.param("kkanbu_tf/sensor1/pose_y", sensor1_pose_y_, 0.0);
    nh_.param("kkanbu_tf/sensor1/pose_z", sensor1_pose_z_, 0.0);

    // Init State
    // tf::Quaternion q;
    map2ego_q_.setRPY(0, 0, ego_init_yaw_);
    map2ego_q_.normalize();

    // tf::Transform transform;
    map2ego_transform_.setOrigin(tf::Vector3(ego_init_x_,ego_init_y_, 0.0));
    map2ego_transform_.setRotation(map2ego_q_);

    // Sunscriber
    // sub_VehicleState = nh_.subscribe("/ego/vehicle_state", 10, &TF_Broadcaster::get_VehicleState, this);
    sub_VehicleState = nh_.subscribe("/current_state", 10, &TF_Broadcaster::get_VehicleState, this);
}

TF_Broadcaster::~TF_Broadcaster() {}

// void TF_Broadcaster::get_VehicleState(const kkanbu_msgs::VehicleState::ConstPtr& msg){
//   ego_state_ = *msg;
//   isStateUpdate = true;
// }

void TF_Broadcaster::get_VehicleState(const kkanbu_msgs::current_state::ConstPtr& msg){
  ego_state_ = *msg;
  ego_state_.yaw = -ego_state_.yaw; 
  isStateUpdate = true;
}


void TF_Broadcaster::Update_Ego_TF(){
  if(isStateUpdate){
    printf("Current Yaw : %f\n", ego_state_.yaw*180/PI);
    map2ego_q_.setRPY(0, 0, ego_state_.yaw);
    map2ego_q_.normalize();
    map2ego_transform_.setOrigin(tf::Vector3(ego_state_.x,ego_state_.y, 0.0));
    map2ego_transform_.setRotation(map2ego_q_);
  }else{
    ROS_WARN("Vehicle State Not Updated!");
  }
}

void TF_Broadcaster::broadcasting_mapToEgo_tf() {
    
    Update_Ego_TF();

    broadcast_TF.sendTransform(tf::StampedTransform(
        map2ego_transform_, ros::Time::now(), "/map", "/ego"));
}

void TF_Broadcaster::broadcasting_egoToSensor_tf() {
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  tf::Transform transform;
  // Extrinsic Calibration
  transform.setOrigin(tf::Vector3(sensor1_pose_x_,sensor1_pose_y_,sensor1_pose_z_));
  transform.setRotation(q);

  // broadcasting the vehicle's body coordinate system
  // The parent coordinate is ego, child coordinate is sensor.
  broadcast_TF.sendTransform(tf::StampedTransform(
      transform, ros::Time::now(), "/ego", "/sensor1"));
}

void TF_Broadcaster::broadcast_all_tf(){
    broadcasting_mapToEgo_tf();
    broadcasting_egoToSensor_tf();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "kkanbu_tf");
  TF_Broadcaster br;

  // The approximate control time is 100 Hz
  ros::Rate loop_rate(100);
  
  while (ros::ok()) {
    br.broadcast_all_tf();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}