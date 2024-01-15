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

struct StoreData {
    float north;
    float east;
    float down;
    float heading;     // robot heading[radian]
    uint32_t utm_zone;
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
StoreData data;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Fill data with Odometry data
    data.utm_x = msg->pose.pose.position.x;
    data.utm_y = msg->pose.pose.position.y;
}

void liorfCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Fill data with Odometry data
    data.utm_x = msg->pose.pose.position.x;
    data.utm_y = msg->pose.pose.position.y;
}

void navCallback(const interfaces_msgs::AutoNavigation::ConstPtr& msg) {

    float mil2deg = 0.0573;
    float deg2rad = 3.141592/180.0;

    float heading = ((msg->nav_detail.heading)*mil2deg)*deg2rad;

    data.utm_zone =  msg->nav_detail.zone_char.utmzone_u;
    data.utm_letter = msg->nav_detail.zone_char.utmzone;

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
    //ros::Subscriber odom_sub = nh.subscribe("/localization", 10, odomCallback);

    // Subscribe to the Odometry topic
    ros::Subscriber lio_sub = nh.subscribe("/liorf/mapping/odometry", 10, liorfCallback);

    // Subscribe to the Navigation topic
    ros::Subscriber nav_sub = nh.subscribe("/nav_data", 10, navCallback);

    // UDP information
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(12301); // my pc port: 22301 & other pc port: 12301
    serverAddr.sin_addr.s_addr = inet_addr("192.168.4.12"); // IP number

    // UDP socket
    int udpSocket = socket(AF_INET, SOCK_DGRAM, 0);

    ros::Rate loop_rate(50);  // 50Hz

    int count = 0;
    float init_heading = 0.0;
    float init_heading_com = 0.0;
    float init_east = 0.0;
    float init_north = 0.0;

    float cos_heading = 0.0;
    float sin_heading = 0.0;
    float deg2rad = 3.141592/180.0;
    float rad2deg = 180.0/3.141592;

    float bias = 4.7581;           // LiDAR & Nav bias
    //float bias = 0.0;           // LiDAR & Nav bias
    float init_slam_x = 0.0;
    float init_slam_y = 0.0;

    while (ros::ok()) {
        // Create a LocationData struct and initialize it
        LocationData udp_data;
        memset(&udp_data, 0, sizeof(LocationData));

        //std::cout<< sizeof(LocationData) << std::endl;
        std::strcpy(udp_data.data_type, "LiSL");    // Set data_type

        udp_data.h1 = 1;
        udp_data.h2 = 1;
        udp_data.h3 = 1;
        udp_data.h4 = 27;
        udp_data.h5 = 27;
        udp_data.error_flag = 0;

        // init data && map flag matching

        int map_flag = 0;

        // init data && map flag matching
        //if(map_flag == 1 && count && 속도 < 1.0){             // Map matching succed
        if(map_flag == 1 && count < 30){                       // Map matching succed
            init_heading = (-159.67)*(3.141592/180.0);
            init_east = 321259.48;
            init_north = 4155175.36;                        

            // 초기화 중에는 slam 데이터 error flag 1로 구성
            udp_data.error_flag = 1;
        }
        //else if(map_flag == 0 && count < 100 && 속도 < 1.0){  // Map matching fail
        
        if(map_flag == 0 && count < 30){                       // Map matching fail
            init_heading = (data.heading*rad2deg + bias)*(3.141592/180.0);
            init_east = data.east;
            init_north = data.north;

            init_slam_x = data.utm_x;
            init_slam_y = data.utm_y;

            std::cout << "This is" << std::endl;
            std::cout << " Init heading    :: " << init_heading*(180/3.141592) << std::endl;

            // 초기화 중에는 slam 데이터 error flag 1로 구성
            udp_data.error_flag = 1;
        }

    
        // SLAM x,y -> UTM north, east
        cos_heading = cos(init_heading); sin_heading = sin(init_heading);
        
        matRot(0,0) = cos_heading; matRot(1,1) = cos_heading;       //Rotation cos
        matRot(0,1) = (-1.0)*sin_heading; matRot(1,0) = sin_heading;  //Rotation sin     

        vecxy(0) = data.utm_x - init_slam_x;
        vecxy(1) = -data.utm_y - init_slam_y;

        vecNE = matRot*vecxy;

        // Copy the subscribed data into udp_data    
        udp_data.north = vecNE(0);
        udp_data.east = vecNE(1);
        udp_data.down = data.down;        
        udp_data.heading = data.heading;        

        // This value should change with Global.pcd(0,0,0)'s UTM x, y, zone, letter
        //udp_data.utm_zone = 52;
        //udp_data.utm_letter = 'S';
        udp_data.utm_zone = data.utm_zone;
        udp_data.utm_letter = data.utm_letter;
        udp_data.utm_x = init_east;     // Replace with the actual UTM X value        
        udp_data.utm_y = init_north;    // Replace with the actual UTM Y value

        // Value check        
        std::cout << " Init heading    :: " << init_heading*(180/3.141592) << std::endl;
        std::cout << " Heading         :: " << udp_data.heading*(180/3.141592) << std::endl;
        std::cout << " Udp_data.east   :: " << udp_data.east << std::endl;
        std::cout << " Udp_data.north  :: " << udp_data.north << std::endl;

        std::cout << " data.utm_zone   :: " << data.utm_zone << std::endl;
        std::cout << " data.utm_letter :: " << data.utm_letter << std::endl;

        std::cout << " Nav east error  :: " << (data.east - (udp_data.east + init_east))<< std::endl;
        std::cout << " Nav north error :: " << (data.north - (udp_data.north + init_north))<< std::endl;
            
        // Send data
        sendto(udpSocket, &udp_data, sizeof(udp_data), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

        std::cout << "========== UDP data send ==========" << std::endl; 
        std::cout << std::endl; 
        count++;

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Close socket
    close(udpSocket);

    std::cout << "UDP socket closed" << std::endl;

    return 0;
}