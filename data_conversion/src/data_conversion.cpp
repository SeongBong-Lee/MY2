#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <interfaces_msgs/AutoNavigation.h>
#include "geographic_msgs/GeoPoint.h" // 추가된 헤더
#include <cmath>
#include <GeographicLib/UTMUPS.hpp>
#include "geographic_msgs/GeoPoint.h"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

struct GeoPoint {
    double latitude;
    double longitude;
};

struct StoreData {
    float north;
    float east;
    float down;
    float heading;
    float roll;
    float pitch;
    uint32_t utm_zone;
    char utm_letter;
    float utm_x;
    float utm_y;
};

ros::Publisher imu_pub;
ros::Publisher nav_pub;

sensor_msgs::Imu ouster_imu;
sensor_msgs::Imu hdrt_imu;

void RPYToQuaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion& quat) {
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);

    quat.x = quaternion.x();
    quat.y = quaternion.y();
    quat.z = quaternion.z();
    quat.w = quaternion.w();
}

void quaternionToRPY(const geometry_msgs::Quaternion& quat, double& roll, double& pitch, double& yaw) {
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (quat.w * quat.x + quat.y * quat.z);
    double cosr_cosp = 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (quat.w * quat.y - quat.z * quat.x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void ouster_imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ouster_imu = *msg;

    //sensor_msgs::Imu modified_imu = ouster_imu;
    //imu_pub.publish(modified_imu); // publish modified data
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    hdrt_imu = *msg; // message copy    
}

GeoPoint utm2lla(double xx, double yy, int utmzone_num, int utmzone_let) {
    GeoPoint lla;

    // 수식에 사용되는 상수들
    const double sa = 6378137.0;
    const double sb = 6356752.314245;
    const double e2 = sqrt((sa * sa) - (sb * sb)) / sb;
    const double e2cuadrada = e2 * e2;
    const double c = (sa * sa) / sb;

    const double X = xx - 500000.0;
    double Y = yy;

    const double S = (utmzone_num * 6.0) - 183.0;
    const double lat = Y / (6366197.724 * 0.9996);
    const double v = (c / sqrt(1 + (e2cuadrada * cos(lat) * cos(lat)))) * 0.9996;
    const double a = X / v;
    const double a1 = sin(2 * lat);
    const double a2 = a1 * cos(lat) * cos(lat);
    const double j2 = lat + (a1 / 2.0);
    const double j4 = (3 * j2 + a2) / 4.0;
    const double j6 = (5 * j4 + a2 * cos(lat) * cos(lat)) / 3.0;
    const double alfa = 3.0 / 4.0 * e2cuadrada;
    const double beta = 5.0 / 3.0 * pow(alfa, 2);
    const double gama = 35.0 / 27.0 * pow(alfa, 3);
    const double Bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
    const double b = (Y - Bm) / v;
    const double Epsi = (e2cuadrada * a * a) / 2.0 * cos(lat) * cos(lat);
    const double Eps = a * (1 - (Epsi / 3.0));
    const double nab = b * (1 - Epsi) + lat;
    const double senoheps = (exp(Eps) - exp(-Eps)) / 2.0;
    const double Delt = atan(senoheps / cos(nab));
    const double TaO = atan(cos(Delt) * tan(nab));
    const double longitude = (Delt * (180.0 / M_PI)) + S;
    const double latitude = (lat + (1 + e2cuadrada * cos(lat) * cos(lat) - 3.0 / 2.0 * e2cuadrada * sin(lat) * cos(lat) * (TaO - lat)) *
                            (TaO - lat)) * (180.0 / M_PI);

    lla.latitude = latitude;
    lla.longitude = longitude;

    return lla;
}


void navCallback(const interfaces_msgs::AutoNavigation::ConstPtr& msg) {
    float mil2deg = 0.0573;
    float deg2rad = 3.141592 / 180.0;
    float heading = ((msg->nav_detail.heading) * mil2deg) * deg2rad;

    StoreData data; // StoreData 구조체 선언
    data.utm_zone = msg->nav_detail.zone_char.utmzone_u;
    data.utm_letter = msg->nav_detail.zone_char.utmzone;
    data.heading = heading;
    data.east = msg->nav_detail.east;
    data.north = msg->nav_detail.north;
    data.down = msg->nav_detail.elevation;

/*     std::cout << "data.utm_zone : " << data.utm_zone << std::endl;
    std::cout << "data.utm_letter : " << data.utm_letter << std::endl;
    std::cout << "data.east : " << data.east << std::endl;
    std::cout << "data.north : " << data.north << std::endl;  */

    // UTM to LLA 변환
    GeoPoint result = utm2lla(data.east, data.north, data.utm_zone, data.utm_letter);

    // NavSatFix 메시지 설정
    sensor_msgs::NavSatFix output_nav;
    //output_nav.header = msg->header;
    output_nav.latitude = result.latitude; // 변환된 위도 값
    output_nav.longitude = result.longitude; // 변환된 경도 값
    output_nav.altitude = data.down; // 변환된 고도 값

    nav_pub.publish(output_nav); // 변환된 NavSatFix 메시지 publish

    std::cout << "NAV Modified ~ing\n" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_conversion");
    ros::NodeHandle nh;

    ros::Subscriber ouster_imu_sub = nh.subscribe("/ouster/imu", 10, ouster_imuCallback);

    ros::Subscriber imu_sub = nh.subscribe("/imu_data", 10, imuCallback);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/output_imu_data", 10);

    ros::Subscriber nav_sub = nh.subscribe("/nav_data", 10, navCallback);
    nav_pub = nh.advertise<sensor_msgs::NavSatFix>("/output_nav_data", 10);

    ros::Rate loop_rate(40);

    while (ros::ok()) {
        ros::spinOnce();

        sensor_msgs::Imu modified_imu = ouster_imu;

        //Change Time
        //modified_imu.header.stamp = ouster_imu->header.stamp;
        //modified_imu.header.seq = ouster_imu->header.seq;
        //modified_imu.header.frame_id = ouster_imu->frame_id

        // Change frame id
        //modified_imu.header.frame_id = "os_imu";

        // y and z axis sign change
         modified_imu.angular_velocity.y = -hdrt_imu.angular_velocity.y;
        modified_imu.angular_velocity.z = -hdrt_imu.angular_velocity.z;
        modified_imu.linear_acceleration.y = -hdrt_imu.linear_acceleration.y;
        modified_imu.linear_acceleration.z = -hdrt_imu.linear_acceleration.z;        

        // quaternion 2 RPY
        double roll, pitch, yaw;
        quaternionToRPY(hdrt_imu.orientation, roll, pitch, yaw);

        float rad2deg = 180/3.141592;
        float deg2mil = 1.0/0.0573;

        //std::cout << "roll : " << roll*rad2deg*deg2mil << std::endl;
        //std::cout << "pitch : " << pitch*rad2deg*deg2mil << std::endl;
        //std::cout << "yaw : " << yaw*rad2deg*deg2mil << std::endl;

        // RPY 2 quaternion
        double x, y, z, w;
        geometry_msgs::Quaternion modified_quat;
        RPYToQuaternion(roll, pitch, yaw, modified_quat);

        modified_imu.orientation = modified_quat;

        std::cout << "xx : " << (hdrt_imu.orientation.x - modified_quat.x) << std::endl;
        std::cout << "yy : " << (hdrt_imu.orientation.y - modified_quat.y) << std::endl;
     

        //Covariance
         for (int i = 0; i < 9; ++i) {
            modified_imu.orientation_covariance[i] = 0.0;
        }
        modified_imu.orientation_covariance[0] = 0.01;
        modified_imu.orientation_covariance[4] = 0.01;
        modified_imu.orientation_covariance[8] = 0.01;

        for (int i = 0; i < 9; ++i) {
            modified_imu.angular_velocity_covariance[i] = 0.0;
        }
        modified_imu.angular_velocity_covariance[0] = 0.0006;
        modified_imu.angular_velocity_covariance[4] = 0.0006;
        modified_imu.angular_velocity_covariance[8] = 0.0006;
        
        for (int i = 0; i < 9; ++i) {
            modified_imu.linear_acceleration_covariance[i] = 0.0;
        }
        modified_imu.linear_acceleration_covariance[0] = 0.01;
        modified_imu.linear_acceleration_covariance[4] = 0.01;
        modified_imu.linear_acceleration_covariance[8] = 0.01;
        
        imu_pub.publish(modified_imu); // publish modified data*/

        std::cout << "IMU Modified ~ing\n" << std::endl;

        std::cout << "==================\n" << std::endl;
        loop_rate.sleep();
    }

    return 0;
}


