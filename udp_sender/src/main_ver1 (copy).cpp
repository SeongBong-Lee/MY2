#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen>
#include <interfaces_msgs/AutoNavigation.h>

// Global data structure
#pragma pack(1)
struct LocationData {
    char data_type[4]; // LiDAR SLAM : 'L', 'i', 'S', 'L'
    uint32_t h1;       // 1
    uint32_t h2;       // 1
    uint32_t h3;       // 1
    uint32_t h4;       // 16
    uint32_t h5;       // 16
    bool error_flag;   // if 1 : error & else : normal
    float north;
    float east;
    float down;
    float heading;     // robot heading[radian]
    unsigned char utm_zone;
    char utm_letter;
    float utm_x;
    float utm_y;
};
#pragma pack()
//R_mat, NE_vector [2,2], [2,1]
Eigen::Matrix2f matRot;
Eigen::Vector2f vecNE;
Eigen::Vector2f vecxy;

// Global data structure for storing subscribed data
LocationData data;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Fill data with Odometry data
    data.north = msg->pose.pose.position.x;
    data.east = msg->pose.pose.position.y;
    data.down = msg->pose.pose.position.z;

    float init_heading = (-164.46+4.75)*(3.141592/180.0);
    //float init_heading = (-164.46-38.6575)*(3.141592/180.0);
    float cos_heading;
    float sin_heading;

    
    cos_heading = cos(init_heading); sin_heading = sin(init_heading);

    
    matRot(0,0)=cos_heading; matRot(1,1)=cos_heading; //Rotation cos
    matRot(0,1)=sin_heading; matRot(1,0)=(-1)*sin_heading; //Rotation sin     

    vecxy(0) = -msg->pose.pose.position.y;

    vecxy(1) = msg->pose.pose.position.x;

    vecNE = matRot*vecxy;

    //vecNE(0) = data.north;
    //vecNE(1) = data.east;
}

void navCallback(const interfaces_msgs::AutoNavigation::ConstPtr& msg) {

    float mil2deg = 0.0573;
    float deg2rad = 3.141592/180.0;

    float heading = (-(msg->nav_detail.heading)*mil2deg + 90)*deg2rad;

    // Fill data with Odometry data
    data.heading = heading;  // Replace with the actual heading value

    data.east = msg->nav_detail.east;
    data.north = msg->nav_detail.north;
}


int main(int argc, char **argv) {
    // ROS Init
    ros::init(argc, argv, "udp_sender_node");
    ros::NodeHandle nh;

    // Subscribe to the Odometry topic
    ros::Subscriber odom_sub = nh.subscribe("/localization", 10, odomCallback);

    // Subscribe to the Navigation topic
    ros::Subscriber nav_sub = nh.subscribe("/nav_data", 10, navCallback);

    // UDP information
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(12301); // my pc port: 22301 & other pc port: 12301
    serverAddr.sin_addr.s_addr = inet_addr("172.21.1.12"); // IP number

    // UDP socket
    int udpSocket = socket(AF_INET, SOCK_DGRAM, 0);

    ros::Rate loop_rate(50);  // 50Hz

    while (ros::ok()) {
        // Create a LocationData struct and initialize it
        LocationData udp_data;
        memset(&udp_data, 0, sizeof(LocationData));

        std::cout<< sizeof(LocationData) << std::endl;
        std::strcpy(udp_data.data_type, "LiSL");    // Set data_type

        udp_data.h1 = 1;
        udp_data.h2 = 1;
        udp_data.h3 = 1;
        udp_data.h4 = 27;
        udp_data.h5 = 27;
        udp_data.error_flag = false;

        // Copy the subscribed data into udp_data

        
        udp_data.north = vecNE(1);
        udp_data.east = vecNE(0);
        udp_data.down = data.down;
        
        udp_data.heading = data.heading;        

        // This value should change with Global.pcd(0,0,0)'s UTM x, y, zone, letter
        udp_data.utm_zone = 52;         // Replace with the actual UTM zone
        udp_data.utm_letter = 'S';      // Replace with the actual UTM letter
        udp_data.utm_x = 321259.48;        // Replace with the actual UTM X value        
        udp_data.utm_y = 4155175.36;       // Replace with the actual UTM Y value

        // Value check
        std::cout << "heading        :: " << udp_data.heading*(180/3.141592) << std::endl;
        std::cout << "udp_data.east  :: " << udp_data.east << std::endl;
        std::cout << "udp_data.north :: " << udp_data.north << std::endl;

        std::cout << " nav east  error :: " << (data.east - (udp_data.east + 321259.48))<< std::endl;
        std::cout << " nav north  error :: " << (data.north - (udp_data.north + 4155175.36))<< std::endl;

        

        std::cout << "udp_data.east  :: " << udp_data.east +  321259.48<< std::endl;
        std::cout << "udp_data.north :: " << udp_data.north + 4155175.36 << std::endl;
            

        // Send data
        sendto(udpSocket, &udp_data, sizeof(udp_data), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

        std::cout << "========== UDP data send ==========" << std::endl; 
        std::cout << std::endl; 

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Close socket
    close(udpSocket);

    std::cout << "UDP socket closed" << std::endl;

    return 0;
}