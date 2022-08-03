#include "kkanbu_pose.hpp"

poseEstimation::poseEstimation()
:isInitGNSS(false),isInitIMU(false)
{
    int buffer_size = 5;
    GNSS_sub = nh.subscribe("/ublox_msgs/navpvt", buffer_size, &poseEstimation::GNSS_Callback, this);
    IMU_sub = nh.subscribe("/imu/data", buffer_size, &poseEstimation::IMU_Callback, this);
    nh.param<double>("kkanbu_pose/initial_yaw", current_state.yaw, 0.0);

    m_pub_current_state = nh.advertise<kkanbu_msgs::current_state>("/current_state", buffer_size);
}
poseEstimation::~poseEstimation()
{
}

void poseEstimation::GNSS_Callback(const ublox_msgs::NavPVT::ConstPtr &msg) {

    if(!isInitGNSS){
        double latitude = (double)(msg->lat)/10000000.0;
        double longitude = (double)(msg->lon)/10000000.0;

        init_local = convert_to_map_frame(latitude,longitude);
        gnss_prev_time = ros::Time::now().toSec();

        prev_local = init_local;
        current_state.x = 0;
        current_state.y = 0;

        isInitGNSS = true;
    }
    
    double latitude = (double)(msg->lat)/10000000.0;
    double longitude = (double)(msg->lon)/10000000.0;

 
    local = convert_to_map_frame(latitude, longitude);

    double gnss_c_time = ros::Time::now().toSec();




    // std::cout << "time ::: "  << (c_time-prev_time)<< std::endl;

    distance = sqrt(pow(local.y - prev_local.y, 2) + pow(local.x - prev_local.x, 2));
    current_state.velocity = distance / (gnss_c_time-gnss_prev_time);
    std::cout << "velocity : " << current_state.velocity  << std::endl;

    if (current_state.velocity > 0.8)
    {
    
        // if ((local.x - prev_local.x)>0.1 && (local.y - prev_local.y)>0.1){  
            heading_deg = atan2(local.y - prev_local.y, local.x - prev_local.x);
            // std::cout << "===========================atan2===========================" << heading_deg << std::endl;
            heading_deg = -heading_deg*180.0/M_PI;

            if(heading_deg > 360){
                heading_deg -= 360;
            }
            if(heading_deg > 180){
                heading_deg -= 360;
            }
            if(heading_deg < -360){
                heading_deg += 360;
            }
            if(heading_deg < -180){
                heading_deg += 360;    
            }
            std::cout << "=========================== Estimated Pose ===========================\n"
                            <<"yaw[deg]: " << heading_deg << std::endl;
        
            current_state.yaw = heading_deg;
            prev_yaw = heading_deg;
        // }

    }


    current_state.x = local.x;
    current_state.y = local.y;
    
    // std::cout << "x : "<< current_state.x << " y : "  << current_state.y << std::endl;
    
    prev_local = local;
    gnss_prev_time = gnss_c_time;

    return;
}


struct poseEstimation::local poseEstimation::convert_to_map_frame(double lat, double lon)
{
    double dKappaLat = 0;
    double dKappaLon = 0;  

    double hgt = 0.00001;

    dKappaLat = FnKappaLat( m_dRefLatitude_deg , hgt ); 
    dKappaLon = FnKappaLon( m_dRefLatitude_deg , hgt );

    struct local local_state;

    local_state.x = (lon-m_dRefLongitude_deg)/dKappaLon;
    local_state.y = (lat-m_dRefLatitude_deg)/dKappaLat;

    return local_state;
}


double poseEstimation::FnKappaLat(double dRef_Latitude, double dHeight)
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


double poseEstimation::FnKappaLon(double dRef_Latitude, double dHeight)
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

void poseEstimation::IMU_Callback(const sensor_msgs::Imu:: ConstPtr &msg){

    if(!isInitIMU) {
        distance = 0;
        prev_yaw = current_state.yaw;
        imu_prev_time = ros::Time::now().toSec();
        isInitIMU = true;
    }
    
    current_state.yaw_rate = -((double) msg->angular_velocity.z); // deg/s

    // std::cout << "yaw_rate " << current_state.yaw_rate << std::endl;

    if (current_state.yaw_rate < 0.05 && current_state.yaw_rate > -0.05){
        current_state.yaw_rate = 0;
    }
    
    double imu_c_time = ros::Time::now().toSec();
    
    // current_state.velocity = distance / (imu_c_time-imu_prev_time); // m/s
    current_state.yaw = prev_yaw + (current_state.yaw_rate * RAD2DEG * (imu_c_time-imu_prev_time)); // deg

    // std::cout << "yaw : " << current_state.yaw << std::endl;


    
    if(current_state.yaw > 360){
        current_state.yaw -= 360;
    }
    if(current_state.yaw > 180){
        current_state.yaw -= 360;
    }
    if(current_state.yaw < -360){
        current_state.yaw += 360;    
    }
    if(current_state.yaw < -180){
        current_state.yaw += 360;    
    }



    prev_yaw = current_state.yaw;
    imu_prev_time = imu_c_time;
}

void poseEstimation::publish_currentstate(){

    if(isInitGNSS && isInitIMU){

        kkanbu_msgs::current_state pub_state;
        pub_state.x = current_state.x;
        pub_state.y = current_state.y;
        pub_state.velocity = current_state.velocity;
        pub_state.yaw = current_state.yaw*M_PI/180.0;
        pub_state.yaw_rate = current_state.yaw_rate;

        m_pub_current_state.publish(pub_state);
    }
    else{
        std::cout << "No Sensor!!" << std::endl;
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "pose_estimation");
    ros::NodeHandle nh;

    poseEstimation poseEstimation;

    int loop_hz = 10;
    ros::Rate loop_rate(loop_hz);

    while(ros::ok())
    {
        
        ros::spinOnce();

        poseEstimation.publish_currentstate();

        loop_rate.sleep();
    }
    return 0;

}
