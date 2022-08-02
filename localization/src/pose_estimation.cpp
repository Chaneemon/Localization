# include "pose_estimation.hpp"

poseEstimation::poseEstimation()
:isInitGNSS(false), gnssExistFlag(false), vehExistFlag(false), imuExistFlag(false), isFirstPrediction(true), isOnlyPrediction(false), isOnlyGNSS(false)
{
    int buffer_size = 10;
    GNSS_sub = nh.subscribe("/ublox_msgs/navpvt", buffer_size, &poseEstimation::GNSS_Callback, this);
    IMU_sub = nh.subscribe("/imu/data",buffer_size, &poseEstimation::IMU_Callback, this);
    VEH_sub = nh.subscribe("/vehicle_state",buffer_size, &poseEstimation::VEH_Callback, this);

    m_pub_gnss_enu_pose = nh.advertise<geometry_msgs::PoseStamped>("/gnssENUPose", buffer_size);
    m_pub_estimated_pose = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", buffer_size);

}
poseEstimation::~poseEstimation()
{
}

void poseEstimation::Run()
{
    if(isInitGNSS)
    {
        geometry_msgs::PoseStamped gnssEnuPose;

        gnssEnuPose = convert_to_map_frame(gnss.latitude, gnss.longitude, gnss.height);

        gnssEnuPose.header.stamp = ros::Time::now();
        gnssEnuPose.header.frame_id = "map";

        tf::Quaternion q_gps;
        q_gps.setRPY(0, 0, gnss.heading);

        geometry_msgs::Quaternion quat_gps;
        geometry_msgs::Quaternion quat_gps_;
        tf::quaternionTFToMsg(q_gps, quat_gps);

        quat_gps_ = tf::createQuaternionMsgFromYaw(gnss.heading);
        //gnssEnuPose.pose.orientation = quat_gps;
        gnssEnuPose.pose.orientation = quat_gps_;

        m_pub_gnss_enu_pose.publish(gnssEnuPose);
        if(_DEBUG_MODE)
        {
            // std::cout << "========================= Published GPS LLH =========================\n"
            // <<"gnss_latitude: " << gnss.latitude << " gnss_longitude: " << gnss.longitude
            // << "\ngnss_height: " << gnss.height << std::endl;
            
            //std::cout << "========================= Published GPS Pose =========================\n"
            //<<"Pub east [m]: " << gnssEnuPose.pose.position.x << " Pub north [m]: " << gnssEnuPose.pose.position.y
            //<< "\nPub up [m]: " << gnssEnuPose.pose.position.z <<" Pub heading [deg]: " <<gnss.heading* 180.0 / M_PI<<std::endl;
        }
    }
    // PUb only GNSS
    if(isOnlyGNSS){
        geometry_msgs::PoseStamped estimatedPose;

        estimatedPose.header.frame_id = "map";
        estimatedPose.header.stamp.sec = veh.sec;
        estimatedPose.header.stamp.nsec = veh.nsec;

        estimatedPose = convert_to_map_frame(gnss.latitude, gnss.longitude, gnss.height);
        estimatedPose.pose.position.z = 0.0;
        tf::Quaternion q_state;
        q_state.setRPY(0, 0, gnss.heading);

        geometry_msgs::Quaternion quat_state;
        geometry_msgs::Quaternion quat_state_;
        tf::quaternionTFToMsg(q_state, quat_state);

        quat_state_ = tf::createQuaternionMsgFromYaw(gnss.heading);

        // estimatedPose.pose.orientation = quat_state;
        estimatedPose.pose.orientation = quat_state_;

        m_pub_estimated_pose.publish(estimatedPose);
       

    }else{


        if (isInitGNSS && vehExistFlag && imuExistFlag){
            if (isFirstPrediction){
                egoVehicleState[0] = init_gnss_enu.pose.position.x;
                egoVehicleState[1] = init_gnss_enu.pose.position.y;
                egoVehicleState[2] = init_gnss_enu.pose.position.z;
                egoVehicleState[3] = init_gnss.heading;
                isFirstPrediction = false;
            }
            geometry_msgs::PoseStamped gnssEnuPose;

            gnssEnuPose = convert_to_map_frame(gnss.latitude, gnss.longitude, gnss.height);

            Prediction(egoVehicleState, veh);

            if(!isOnlyPrediction){
                MeasurementUpdate(egoVehicleState, P_post, gnss, veh);
            }

            geometry_msgs::PoseStamped estimatedPose;
            geometry_msgs::PoseStamped estimatedPose_;
            estimatedPose.header.frame_id = "map";
            estimatedPose.header.stamp.sec = veh.sec;
            estimatedPose.header.stamp.nsec = veh.nsec;

            estimatedPose.pose.position.x = egoVehicleState[0];
            estimatedPose.pose.position.y = egoVehicleState[1];
            estimatedPose.pose.position.z = 0.0;
            tf::Quaternion q_state;
            q_state.setRPY(0, 0, egoVehicleState[3]);

            geometry_msgs::Quaternion quat_state;
            geometry_msgs::Quaternion quat_state_;
            tf::quaternionTFToMsg(q_state, quat_state);

            // estimatedPose.pose.orientation = quat_state;
            quat_state_ = tf::createQuaternionMsgFromYaw(gnss.heading);

            // estimatedPose.pose.orientation = quat_state;
            estimatedPose.pose.orientation = quat_state_;

            m_pub_estimated_pose.publish(estimatedPose);
        
            if(_DEBUG_MODE)
            {
                //std::cout<<"previous timestamp: "<<prevTimestamp_ms<<std::endl;
                //std::cout << "=========================== Estimated Pose ===========================\n"
                //<< "east[m]: " << egoVehicleState[0] << " north[m]: " << egoVehicleState[1] << " heading[deg]: " << egoVehicleState[3]*180/M_PI << std::endl;
            }

        }
    }


}

void poseEstimation::GNSS_Callback(const ublox_msgs::NavPVT::ConstPtr &msg) {

    if(!isInitGNSS){
        init_gnss.sec = toUtcSeconds(*msg);
        init_gnss.nsec = msg->nano;
        init_gnss.latitude = (double)(msg->lat)/10000000.0;
        init_gnss.longitude = (double)(msg->lon)/10000000.0;
        // init_gnss.latitude = (double)(msg->latitude);
        // init_gnss.longitude = (double)(msg->longitude);
        // init_gnss.height = (double)(msg->altitude);
        heading_deg = (double)(msg->heading)*1e-5;
        heading_deg = -heading_deg;
        heading_deg = heading_deg + 90.0;
        if (heading_deg > 360){
            heading_deg = heading_deg-360;
        }
        if (heading_deg > 180.0){
            heading_deg = heading_deg - 360;
        }
        init_gnss.heading = heading_deg * M_PI / 180.0;
        init_gnss.headAcc = (double)(msg->hAcc) / 1000.f;
        init_gnss.gnssFlag = (int)(msg->flags);
        init_gnss.pDOP = (double)(msg->pDOP);
        init_gnss.numSV = (int)(msg->numSV);
        init_gnss.hAcc = (double)(msg->headAcc)*1e-5f;

        init_gnss_enu = convert_to_map_frame(init_gnss.latitude, init_gnss.longitude, init_gnss.height);
        


        if(_DEBUG_MODE && _DISP_INPUT)
        {
            //ROS_INFO_STREAM("========================= Init GNSS =========================\n");
            //ROS_INFO_STREAM("[Init GNSS] lat_deg: "<<init_gnss.latitude<<", lon_deg: "<<init_gnss.longitude << ", heading_deg: "<<init_gnss.heading);
            // <<"time" << init_gnss.timestamp << std::endl;
        }
        prev_pose = init_gnss_enu;
        isInitGNSS = true;
    }

    gnssExistFlag = true;

    gnss.sec = toUtcSeconds(*msg);
    gnss.nsec = msg->nano;
    gnss.latitude = (double)(msg->lat)/10000000.0;
    gnss.longitude = (double)(msg->lon)/10000000.0;
    // gnss.latitude = (double)(msg->latitude);
    // gnss.longitude = (double)(msg->longitude);
    // ROS_INFO_STREAM("========================= Init GNSS =========================\n");
    // ROS_INFO_STREAM("[Init GNSS] lat_deg: "<<gnss.latitude<<", lon_deg: "<<gnss.longitude);
      
    // gnss.height = (double)(msg->altitude);
    heading_deg = (double)(msg->heading)*1e-5;
    heading_deg = -heading_deg;
    heading_deg = heading_deg + 90.0;
    ROS_INFO_STREAM("[Init GNSS] heading_deg: "<<heading_deg);
        
    gnss.headAcc = (double)(msg->hAcc) / 1000.f;
    gnss.gnssFlag = (int)(msg->flags);
    gnss.pDOP = (double)(msg->pDOP);
    gnss.numSV = (int)(msg->numSV);
    gnss.hAcc = (double)(msg->headAcc)*1e-5f;

    gnss_enu = convert_to_map_frame(gnss.latitude, gnss.longitude, gnss.height);

    double distance = sqrt(pow(gnss_enu.pose.position.y - prev_pose.pose.position.y, 2) +
                                pow(gnss_enu.pose.position.x - prev_pose.pose.position.x, 2));
    std::cout << "distance : " << distance << std::endl;

    if (distance > 0.02)
    {
        if ((gnss_enu.pose.position.x - prev_pose.pose.position.x)>0.01 && (gnss_enu.pose.position.y - prev_pose.pose.position.y)>0.01){  
            heading_deg = atan2(gnss_enu.pose.position.y - prev_pose.pose.position.y, gnss_enu.pose.position.x - prev_pose.pose.position.x);
            heading_deg = heading_deg*180.0/M_PI;
            std::cout << "=========================== Estimated Pose ===========================\n"
                        <<"yaw[deg]: " << heading_deg << std::endl;
        }
    }
    if (heading_deg > 360){
        heading_deg = heading_deg-360;
    }
    if (heading_deg > 180.0){
        heading_deg = heading_deg - 360;
    }
    gnss.heading = heading_deg*M_PI/180.0;

    prev_pose = gnss_enu;


    // geometry_msgs::PoseStamped gnssEnuPose;

    // gnssEnuPose = convert_to_map_frame(gnss.latitude, gnss.longitude, gnss.height);

    // gnssEnuPose.header.stamp = ros::Time::now();
    // gnssEnuPose.header.frame_id = "map";

    // tf::Quaternion q_gps;
    // q_gps.setRPY(0, 0, gnss.heading);

    // geometry_msgs::Quaternion quat_gps;
    // tf::quaternionTFToMsg(q_gps, quat_gps);

    // gnssEnuPose.pose.orientation = quat_gps;


    // m_pub_gnss_enu_pose.publish(gnssEnuPose);
    // if(_DEBUG_MODE)
    // {
    //     // std::cout << "========================= Published GPS LLH =========================\n"
    //     // <<"gnss_latitude: " << gnss.latitude << " gnss_longitude: " << gnss.longitude
    //     // << "\ngnss_height: " << gnss.height << std::endl;
        
    //     std::cout << "========================= Published GPS Pose =========================\n"
    //     <<"Pub east [m]: " << gnssEnuPose.pose.position.x << " Pub north [m]: " << gnssEnuPose.pose.position.y
    //     << "\nPub up [m]: " << gnssEnuPose.pose.position.z <<" Pub heading [deg]: " <<gnss.heading* 180.0 / M_PI<<std::endl;
    // }

    if(_DEBUG_MODE && _DISP_INPUT)
    {
        //ROS_INFO_STREAM("[GNSS] lat_deg: "<<gnss.latitude<<", lon_deg: "<<gnss.longitude << ", heading_deg: "<<gnss.heading);
    }

    return;
}

void poseEstimation::VEH_Callback(const kusv_msgs::State::ConstPtr &msg){

    vehExistFlag = true;

    double timestamp = (double)msg->header.stamp.sec * 1e6 + (double)msg->header.stamp.nsec / 1e3;
    double velocity_kmh = (double)msg->speed_kmh;
    veh.velocity_ms = (double)msg->speed_ms;

    if(_DEBUG_MODE && _DISP_INPUT){
        //ROS_INFO_STREAM("[VEH] speed [km/h]: "<<velocity_kmh);
    }
    return;
}

void poseEstimation::IMU_Callback(const sensor_msgs::Imu::ConstPtr &msg){
    imuExistFlag = true;

    double timestamp = (double)msg->header.stamp.sec * 1e6 + (double)msg->header.stamp.nsec / 1e3;
    veh.timestamp = timestamp;

    angular_velocity_x = msg->angular_velocity.x;
    angular_velocity_y = msg->angular_velocity.y;
    angular_velocity_z = msg->angular_velocity.z;
    
    linear_acceleration_x = msg->linear_acceleration.x;
    linear_acceleration_y = msg->linear_acceleration.y;
    linear_acceleration_z = msg->linear_acceleration.z;

    veh.roll_rate_rads = angular_velocity_x;
    veh.pitch_rate_rads = angular_velocity_y;
    veh.yaw_rate_rads = angular_velocity_z;
    if (_DEBUG_MODE && _DISP_INPUT){
        //ROS_INFO_STREAM("[IMU]  yaw_rate: "<< angular_velocity_z);
    }

    return;
}

geometry_msgs::PoseStamped poseEstimation::convert_to_map_frame(double lat, double lon, double hgt)
{
    double dKappaLat = 0;
    double dKappaLon = 0;  
    //double m_dRefLatitude_deg = 37.540032;
    //double m_dRefLongitude_deg = 127.0709544; //konkuk
    double m_dRefLatitude_deg = 37.2385024; //k-city
     double m_dRefLongitude_deg = 126.7708849;

    // double m_dRefLatitude_deg = init_gnss_gt.latitude;
    // double m_dRefLongitude_deg = init_gnss_gt.longitude;

    // hgt = init_gnss.height;
    if(hgt == 0.0){
        hgt = 0.00001;
    }

    dKappaLat = FnKappaLat( m_dRefLatitude_deg , hgt ); //37.540032, 127.0709544
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

sensor_msgs::NavSatFix poseEstimation::convert_to_llh_frame(float east, float north, float hgt)
{
    double dKappaLat = 0;
    double dKappaLon = 0;  

    //double m_dRefLatitude_deg = 37.540032; //ku
    //double m_dRefLongitude_deg = 127.0709544;
    double m_dRefLatitude_deg = 37.2385024; //k-city
    double m_dRefLongitude_deg = 126.7708849;


    // hgt = init_gnss.height;
    if(hgt == 0.0){
        hgt = 0.00001;
    }

    dKappaLat = FnKappaLat( m_dRefLatitude_deg , hgt );
    dKappaLon = FnKappaLon( m_dRefLatitude_deg , hgt );

    sensor_msgs::NavSatFix pose;
    geometry_msgs::Quaternion quat;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.longitude = east * dKappaLon + m_dRefLongitude_deg;
    pose.latitude = north * dKappaLat + m_dRefLatitude_deg;
    pose.altitude = hgt;


    return(pose);
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


void poseEstimation::Prediction(double state[] , VEHICLE inputIMU)
{
    double predEast_m = state[0];
    double predNorth_m = state[1];
    double predUp_m = state[2];
    double predYaw_rad = state[3];
    

    static double prevTimestamp_ms = inputIMU.timestamp;

    double dt = (inputIMU.timestamp - prevTimestamp_ms)/1e6; // su jeong hae joo sae yo.
    prevTimestamp_ms = inputIMU.timestamp;

    if(dt > 1.0){
        return;
    }

    double east_m = predEast_m + inputIMU.velocity_ms * dt * cos(predYaw_rad);
    double north_m = predNorth_m + inputIMU.velocity_ms * dt * sin(predYaw_rad);
    double Up_m = predUp_m;
    double yaw_rad = predYaw_rad + dt * inputIMU.yaw_rate_rads;

    state[0] = east_m;
    state[1] = north_m;
    state[2] = Up_m;
    state[3] = yaw_rad;
    
    // predEast_m = 
    imuExistFlag = false;
    gnssExistFlag = false;

    if(_DEBUG_MODE)
    {
        //std::cout << "=========================== Predicted Pose ===========================\n"
        //<<"east[m]: " << east_m << " north[m]: " << north_m << " heading[deg]: " << yaw_rad*180/M_PI << std::endl;
        //ROS_INFO_STREAM("DElta TImet" << dt);
        //ROS_INFO_STREAM("VElocity" << inputIMU.velocity_ms *3.6);
    }
}

void poseEstimation::MeasurementUpdate(double state[], double covariance[3][3], GNSS inputGNSS, VEHICLE inputIMU)
{
    double prevEast_m = state[0];
    double prevNorth_m = state[1];
    double prevUp_m = state[2];
    double prevYaw_rad = state[3];

    static double prevTimestamp_ms = inputIMU.timestamp;

    double dt = (inputIMU.timestamp - prevTimestamp_ms)/1e6;
    prevTimestamp_ms = inputIMU.timestamp;

    double velocity_mps = inputIMU.velocity_ms;
    double yawrate_radps = inputIMU.yaw_rate_rads;

    if(dt > 1.0){
        return;
    }

    geometry_msgs::PoseStamped gnssEnu;

    gnssEnu = convert_to_map_frame(gnss.latitude, gnss.longitude, gnss.height);

    double gnssEast_m = gnssEnu.pose.position.x;
    double gnssNorth_m = gnssEnu.pose.position.y;
    double gnssUp_m = gnssEnu.pose.position.z;
    double gnssYaw_rad = gnss.heading;

    double I[3][3] = 
    {
        1., 0., 0.,
        0., 1., 0.,
        0., 0., 1.
    };

    cv::Mat matI(STATE_ORDER, STATE_ORDER, CV_64FC1, I);

    double f[3] =
	{
		prevEast_m + dt*velocity_mps*cos(prevYaw_rad),
		prevNorth_m + dt*velocity_mps*sin(prevYaw_rad),
		prevYaw_rad + dt*yawrate_radps
	};
    cv::Mat matf(STATE_ORDER, 1, CV_64FC1, f);

    if(_DEBUG_MODE)
    {
        //std::cout << "=========================== f mat ===========================\n"
        //<< "east[m]: " << matf.at<double>(0, 0) << " north[m]: " << matf.at<double>(1, 0) << " heading[deg]: " << matf.at<double>(2, 0)*180/M_PI << std::endl;
        //ROS_INFO_STREAM("VElocity" << inputIMU.velocity_ms *3.6);

    }
    double h[3] = 
    {
        prevEast_m,
        prevNorth_m,
        prevYaw_rad
    };
    cv::Mat math(MEAS_ORDER, 1, CV_64FC1, h);

    double z[3] = 
    {
        gnssEast_m,
        gnssNorth_m,
        gnssYaw_rad
    };
    cv::Mat matz(MEAS_ORDER, 1, CV_64FC1, z);

    double H[3][3] = 
    {
        1., 0., 0.,
        0., 1., 0., 
        0., 0., 1.
    };
    cv::Mat matH(MEAS_ORDER, STATE_ORDER, CV_64FC1, H);

    double F[3][3] = 
    {
        1., 0., -dt*velocity_mps*sin(prevYaw_rad),
        0., 1., dt*velocity_mps*cos(prevYaw_rad),
        0., 0., 1.
    };
    cv::Mat matF(STATE_ORDER, STATE_ORDER, CV_64FC1, F);

    cv::Mat matP_post(STATE_ORDER, STATE_ORDER, CV_64FC1, P_post);

    cv::Mat matQ(STATE_ORDER, STATE_ORDER, CV_64FC1, Q);
    cv::Mat matR(MEAS_ORDER, MEAS_ORDER, CV_64FC1, R);

    cv::Mat matP_pri = matF*matP_post*matF.t() + matQ;
    cv::Mat matS = matH*matP_pri*matH.t() + matR;
    cv::Mat invS = matS.inv();
    
    cv::Mat matK = matP_pri*matH.t()*matS.inv();
    cv::Mat matx_pri = matf;

    cv::Mat matx_post = matf + matK*(matz - math);
    matP_post = (matI - matK*matH)*matP_pri;
    
    P_post[0][0] =	matP_post.at<double>(0, 0),
    P_post[1][1] =	matP_post.at<double>(1, 1),
    P_post[2][2] =	matP_post.at<double>(2, 2),

    state[0] = matx_post.at<double>(0,0);
    state[1] = matx_post.at<double>(1,0);
    state[2] = gnssUp_m;
    state[3] = matx_post.at<double>(2,0);
    
    if(_DEBUG_MODE)
    {
        //std::cout<<"previous timestamp: "<<prevTimestamp_ms<<std::endl;
        //std::cout << "=========================== Updated Pose ===========================\n"
        //<< "east[m]: " << state[0] << " north[m]: " << state[1] << " heading[deg]: " << state[3]*180/M_PI << std::endl;
    }
}

int main(int argc, char **argv){
    std::string node_name = "pose_estimation";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    poseEstimation poseEstimation;
    //ROS_INFO_STREAM("HI");

    int loop_hz = 10;

    ros::Rate loop_rate(loop_hz);
    if(_DEBUG_MODE)
    {
        //printf("Debug Mode On\n\n\n\n");
    }
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        poseEstimation.Run();
    }
    return 0;

}
