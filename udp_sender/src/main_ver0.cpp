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

// Global data structure for storing subscribed data
LocationData data;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Fill data with Odometry data
    data.north = msg->pose.pose.position.x;
    data.east = msg->pose.pose.position.y;
    data.down = msg->pose.pose.position.z;
    data.heading = 0.0;  // Replace with the actual heading value
}

int main(int argc, char **argv) {
    // ROS Init
    ros::init(argc, argv, "udp_sender_node");
    ros::NodeHandle nh;

    // Subscribe to the Odometry topic
    ros::Subscriber odom_sub = nh.subscribe("/localization", 10, odomCallback);

    // UDP information
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(12301); // my pc port: 22301 & other pc port: 12301
    serverAddr.sin_addr.s_addr = inet_addr("172.21.1.12"); // IP number

    // UDP socket
    int udpSocket = socket(AF_INET, SOCK_DGRAM, 0);

    ros::Rate loop_rate(50);  // 50Hz

    // remove
    float temp_data = 1.23;

    while (ros::ok()) {
        // Create a LocationData struct and initialize it
        LocationData udp_data;
        memset(&udp_data, 0, sizeof(LocationData));

        std::cout<< sizeof(LocationData) << std::endl;
        std::strcpy(udp_data.data_type, "LiSL");  // Set data_type

        udp_data.h1 = 1;
        udp_data.h2 = 1;
        udp_data.h3 = 1;
        udp_data.h4 = 27;
        udp_data.h5 = 27;
        udp_data.error_flag = false;

        // Copy the subscribed data into udp_data
        /*
        udp_data.north = data.north;
        udp_data.east = data.east;
        udp_data.down = data.down;
        udp_data.heading = data.heading;        
            */
        udp_data.north = 0.0 + temp_data;
        udp_data.east = 1.0;
        udp_data.down = 0.0;
        udp_data.heading = 0.0;        


        udp_data.utm_zone = 52;    // Replace with the actual UTM zone
        udp_data.utm_letter = 'S';  // Replace with the actual UTM letter
        //udp_data.utm_x = 0.0;       // Replace with the actual UTM X value
        udp_data.utm_x = 321254;       // Replace with the actual UTM X value        
        udp_data.utm_y = 4155148;       // Replace with the actual UTM Y value

        //remove
        temp_data+= 0.1;

        //std::cout << temp_data << std :: endl;

        // Send data
        sendto(udpSocket, &udp_data, sizeof(udp_data), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

        std::cout << "UDP data send" << std::endl; 

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Close socket
    close(udpSocket);

    std::cout << "UDP socket closed" << std::endl;

    return 0;
}