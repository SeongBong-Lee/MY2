#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>
#include <interfaces_msgs/AutoNavigation.h>
#include <std_msgs/Char.h>
#include <tf2/LinearMath/Quaternion.h>

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

StoreData data; // StoreData 구조체 정의
StoreData val;

char map_flag = 2;
void mapflagCallback(const std_msgs::Char::ConstPtr& msg) {
    // Fill data with Odometry data
    map_flag = msg->data;
}
geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;

void navCallback(const interfaces_msgs::AutoNavigation::ConstPtr& msg) {

    Eigen::Matrix2f matRot;
    Eigen::Vector2f vecNE;
    Eigen::Vector2f vecxy;

    float mil2deg = 0.0573;
    float deg2rad = 3.141592 / 180.0;
    float heading = ((msg->nav_detail.heading) * mil2deg) * deg2rad;

    data.utm_zone = msg->nav_detail.zone_char.utmzone_u;
    data.utm_letter = msg->nav_detail.zone_char.utmzone;
    data.heading = heading;
    data.roll = ((msg->nav_detail.roll) * mil2deg) * deg2rad;
    data.pitch = ((msg->nav_detail.pitch) * mil2deg) * deg2rad;
    data.north = msg->nav_detail.north;
    data.east = msg->nav_detail.east;
    data.down = msg->nav_detail.elevation;

    //float init_heading = (-159.67)*(3.141592/180.0);
    //float init_east = 321259.48;
    //float init_north = 4155175.36;

    float init_heading = (108.60)*(3.141592/180.0);
    float init_east = 320942.1875;
    float init_north = 4155025.5;
    float init_down = 37.81;

    //Nav -> SLAM    
    float cos_heading = cos(init_heading);
    float sin_heading = sin(init_heading);

    matRot(0, 0) = cos_heading;
    matRot(1, 1) = cos_heading;
    matRot(0, 1) = (-1.0) * sin_heading;
    matRot(1, 0) = sin_heading;

    vecxy(0) = data.north - init_north;
    vecxy(1) = data.east - init_east;


    vecNE = matRot.inverse() * vecxy;

    val.north = vecNE(0);
    val.east = -vecNE(1);

    // 위에서 가져온 정보를 기반으로 initialPoseMsg 업데이트
    initialPoseMsg.pose.pose.position.x = val.north; // north 값을 x로 설정
    initialPoseMsg.pose.pose.position.y = val.east; // east 값을 y로 설정
    initialPoseMsg.pose.pose.position.z = data.down - init_down; // east 값을 z로 설정

    // roll, pitch, yaw 기반으로 orientation 업데이트
    // 여기서는 roll, pitch, yaw(heading) 값을 사용하여 Quaternion을 생성하는 방법을 사용할 수 있습니다.
    // 적절한 방법으로 roll, pitch, yaw를 Quaternion으로 변환하여 설정해야 합니다.
    // 아래는 예시로 Euler 각도를 Quaternion으로 변환하는 방법을 보여줍니다.
    tf2::Quaternion quat;
    quat.setRPY(data.roll, data.pitch, data.heading); // roll, pitch, yaw를 Quaternion으로 변환
    initialPoseMsg.pose.pose.orientation.x = quat.x(); // Quaternion 값을 orientation으로 설정
    initialPoseMsg.pose.pose.orientation.y = quat.y();
    initialPoseMsg.pose.pose.orientation.z = quat.z();
    initialPoseMsg.pose.pose.orientation.w = quat.w(); 
    //initialPoseMsg.pose.pose.orientation.x = 0.0; // Quaternion 값을 orientation으로 설정
    //initialPoseMsg.pose.pose.orientation.y = 0.0;
    //initialPoseMsg.pose.pose.orientation.z = 0.0;
    //initialPoseMsg.pose.pose.orientation.w = 1.0;

    for (int i = 0; i < 36; ++i) {
        initialPoseMsg.pose.covariance[i] = 0.0;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "initialpose_publisher");
    ros::NodeHandle nh;

    // 파일 경로 설정
    std::string configFilePath = "/home/hdrt/hdrt_ws/src/multi_function/config/initial_pose.yaml";

    // Config 파일 읽기
    YAML::Node config = YAML::LoadFile(configFilePath);

    // 토픽 및 메시지 설정
    ros::Publisher initialPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);    

    // Subscribe to the map matching flag
    ros::Subscriber map_flag_sub = nh.subscribe("/map_flag", 10, mapflagCallback);

    // Subscribe to the Navigation topic
    ros::Subscriber nav_sub = nh.subscribe("/nav_data", 10, navCallback);

    // "/localization" 토픽 구독
    //ros::Subscriber localizationSub = nh.subscribe("/localization", 1, localizationCallback);

    // 루프 시작
    ros::Rate loop_rate(20);  // 20Hz로 설정, 맞춰서 사용

    while (ros::ok()) {
        // 메시지 발행
        initialPosePub.publish(initialPoseMsg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}