#include "tf_broadcaster.hpp"

tfBroadcaster::tfBroadcaster()
:m_dMapHeight(15.0)
{
    int buffer_size = 1;
    // m_sub_gps_groundtruth = nh.subscribe("/gnssGTENUPose", buffer_size, &tfBroadcaster::CallBackGPSGT, this);
    m_sub_gps = nh.subscribe("/gnssENUPose", buffer_size, &tfBroadcaster::CallBackGPS, this);
    m_sub_estimated_pose = nh.subscribe("/current_pose", buffer_size, &tfBroadcaster::CallBackEstimatedPose, this);
    
}

tfBroadcaster::~tfBroadcaster()
{

}

void tfBroadcaster::CallBackGPS(const geometry_msgs::PoseStamped::ConstPtr &msg){

    static tf::TransformBroadcaster tf_gnss;

    ros::Time pres_time = msg->header.stamp;

    // Get quternion
    tf::Quaternion quternion_gps;
    quaternionMsgToTF(msg->pose.orientation,quternion_gps);

    // Get offset
    tf::Vector3 offset_gps = tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // brodcast
    tf_gnss.sendTransform( 
        tf::StampedTransform(tf::Transform(quternion_gps, offset_gps), pres_time, "map", "gnss_pose"));

}


// void tfBroadcaster::CallBackDR(const geometry_msgs::PoseStamped::ConstPtr &msg){

//     static tf::TransformBroadcaster tf_dr;

//     ros::Time pres_time = msg->header.stamp;

//     // Get quternion
//     tf::Quaternion quternion_dr;
//     quaternionMsgToTF(msg->pose.orientation,quternion_dr);

//     // Get offset
//     tf::Vector3 offset_dr = tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
//     // brodcast
//     tf_dr.sendTransform( 
//         tf::StampedTransform(tf::Transform(quternion_dr, offset_dr), pres_time, "map", "dr_pose"));

// }
void tfBroadcaster::CallBackEstimatedPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    static tf::TransformBroadcaster tf_estimated;

    ros::Time pres_time = msg->header.stamp;

     // Get quternion
    tf::Quaternion quternion_estimated;
    quaternionMsgToTF(msg->pose.orientation,quternion_estimated);

    // Get offset
    tf::Vector3 offset_estimated = tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    // brodcast
    tf_estimated.sendTransform( 
        tf::StampedTransform(tf::Transform(quternion_estimated, offset_estimated), pres_time, "map", "base_link"));
    tf_estimated.sendTransform( 
        tf::StampedTransform(tf::Transform(quternion_estimated, offset_estimated), pres_time, "map", "velodyne"));

}

geometry_msgs::PoseStamped tfBroadcaster::ConvertToMapFrame(float lat, float lon, float hgt)
{
    double dKappaLat = 0;
    double dKappaLon = 0;  

    double m_dRefLatitude_deg = 37.540032;
    double m_dRefLongitude_deg = 127.0709544;
    // double m_dRefLatitude_deg = init_gnss.latitude;
    // double m_dRefLongitude_deg = init_gnss.longitude;

    hgt = m_dMapHeight;

    dKappaLat = FnKappaLat( m_dRefLatitude_deg , hgt );
    dKappaLon = FnKappaLon( m_dRefLatitude_deg , hgt );

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Quaternion quat;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = (lon-m_dRefLongitude_deg)/dKappaLon;
    pose.pose.position.y = (lat-m_dRefLatitude_deg)/dKappaLat;
    pose.pose.position.z = hgt;

    return(pose);
}

double tfBroadcaster::FnKappaLat(double dRef_Latitude, double dHeight)
{
	double dKappaLat = 0;
	double Denominator = 0;
	double dM = 0;

	// estimate the meridional radius
	Denominator = sqrt(1 - Geod_e2 * pow(sin(dRef_Latitude * DEG2RAD), 2));
	dM = Geod_a * (1 - Geod_e2) / pow(Denominator, 3);

	// Curvature for the meridian
	dKappaLat = 1 / (dM + dHeight) * RAD2DEG;

	return dKappaLat;
}
double tfBroadcaster::FnKappaLon(double dRef_Latitude, double dHeight)
{
	double dKappaLon = 0;
	double Denominator = 0;
	double dN = 0;

	// estimate the normal radius
	Denominator = sqrt(1 - Geod_e2 * pow(sin(dRef_Latitude * DEG2RAD), 2));
	dN = Geod_a / Denominator;

	// Curvature for the meridian
	dKappaLon = 1 / ((dN + dHeight) * cos(dRef_Latitude * DEG2RAD)) * RAD2DEG;

	return dKappaLon;
}

int main(int argc, char **argv)
{
	std::string node_name = "tfBroadcaster";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;

    tfBroadcaster tfBroadcaster;

    ros::spin();
	return 0;
}
