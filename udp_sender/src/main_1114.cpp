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
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Point.h>

#include <vector>
std::vector<float> init_heading_data;
std::vector<float> init_east_data;
std::vector<float> init_north_data;
std::vector<float> init_slam_x_data;
std::vector<float> init_slam_y_data;

// Kalman Filter class for 1D state (e.g., position)
class KalmanFilter {
public:
    KalmanFilter(double initial_estimate, double initial_estimate_error, double process_noise, double measurement_noise) {
        estimate_ = initial_estimate;
        estimate_error_ = initial_estimate_error;
        process_noise_ = process_noise;
        measurement_noise_ = measurement_noise;
    }

    // Update the filter with a new measurement
    void update(double measurement) {
        // Prediction step
        double prediction = estimate_;
        double prediction_error = estimate_error_ + process_noise_;

        // Update step (Kalman gain calculation)
        double kalman_gain = prediction_error / (prediction_error + measurement_noise_);

        // Update the estimate and its error
        estimate_ = prediction + kalman_gain * (measurement - prediction);
        estimate_error_ = (1.0 - kalman_gain) * prediction_error;
    }

    double getEstimate() {
        return estimate_;
    }

private:
    double estimate_;
    double estimate_error_;
    double process_noise_;
    double measurement_noise_;
};

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

struct Vel {
    float vx;
    float vy;
    float vz;
};

struct StoreData {
    float north;
    float east;
    float down;
    float heading;
    uint32_t utm_zone;
    char utm_letter;
    float utm_x;
    float utm_y;
};
#pragma pack()

// Global data structure for storing subscribed data
StoreData data;
StoreData data_liorf;
//StoreData sourceData;
StoreData val;
Vel veldata;

bool kalman_flag = true;

// Kalman Filters for east and north
KalmanFilter kalman_filter_east(0.0, 0.1, 0.01, 0.1);  // Tweak the parameters
KalmanFilter kalman_filter_north(0.0, 0.1, 0.01, 0.1);  // Tweak the parameters

void slam2utm(float init_heading, float init_slam_x, float init_slam_y, StoreData& sourceData) {
    Eigen::Matrix2f matRot;
    Eigen::Vector2f vecNE;
    Eigen::Vector2f vecxy;

    float cos_heading = cos(init_heading);
    float sin_heading = sin(init_heading);

    matRot(0, 0) = cos_heading;
    matRot(1, 1) = cos_heading;
    matRot(0, 1) = (-1.0) * sin_heading;
    matRot(1, 0) = sin_heading;

    vecxy(0) = sourceData.utm_x - init_slam_x;
    vecxy(1) = -sourceData.utm_y - (-init_slam_y);

    vecNE = matRot * vecxy;

    val.north = vecNE(0);
    val.east = vecNE(1);

/*     std::cout << "focusfocusfocusfocusfocusfocusfocusfocusfocusfocusfocusfocusfocusfocusfocus" << std::endl;
    std::cout << "init_slam_x" << init_slam_x << std::endl;
    std::cout << "init_slam_y" << init_slam_y << std::endl;
    std::cout << "sourceData.utm_x" << sourceData.utm_x << std::endl;
    std::cout << "sourceData.utm_y" << sourceData.utm_y << std::endl;
    
    std::cout << "vecxy(0)" << vecxy(0) << std::endl;
    std::cout << "vecxy(1)" << vecxy(1) << std::endl;      
    std::cout << "focusfocusfocusfocusfocusfocusfocusfocusfocusfocusfocusfocusfocusfocusfocus" << std::endl; */
}

LocationData generateUdpData(float init_heading, float init_slam_x, float init_slam_y, float init_east, float init_north, StoreData& sourceData) {
    LocationData udp_data;
    memset(&udp_data, 0, sizeof(LocationData));

    // Set common fields
    std::strcpy(udp_data.data_type, "LiSL");
    udp_data.h1 = 1;
    udp_data.h2 = 1;
    udp_data.h3 = 1;
    udp_data.h4 = 27;
    udp_data.h5 = 27;
    udp_data.error_flag = 0;

    // Use the appropriate data source
    slam2utm(init_heading, init_slam_x, init_slam_y, sourceData);

    // Apply Kalman filter to east and north if kalman_flag is true
    if (kalman_flag) {
        kalman_filter_east.update(val.east);
        kalman_filter_north.update(val.north);
        udp_data.east = kalman_filter_east.getEstimate();
        udp_data.north = kalman_filter_north.getEstimate();
    } else {
        udp_data.east = val.east;
        udp_data.north = val.north;
    }

    udp_data.down = sourceData.down;
    udp_data.heading = sourceData.heading;
    udp_data.utm_zone = sourceData.utm_zone;
    udp_data.utm_letter = sourceData.utm_letter;
    udp_data.utm_x = init_east;
    udp_data.utm_y = init_north;

    return udp_data;
}

void kalmanFlagCallback(const std_msgs::Bool::ConstPtr& msg) {
    kalman_flag = msg->data;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Fill data with Odometry data
    data.utm_x = msg->pose.pose.position.x;
    data.utm_y = msg->pose.pose.position.y;
}

void liorfCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Fill data with Odometry data
    data_liorf.utm_x = msg->pose.pose.position.x;
    data_liorf.utm_y = msg->pose.pose.position.y;
}

void navCallback(const interfaces_msgs::AutoNavigation::ConstPtr& msg) {

    float mil2deg = 0.0573;
    float deg2rad = 3.141592/180.0;

    float heading = ((msg->nav_detail.heading)*mil2deg)*deg2rad;

    data.utm_zone =  msg->nav_detail.zone_char.utmzone_u;
    data.utm_letter = msg->nav_detail.zone_char.utmzone;
    data.heading = heading;
    data.north = msg->nav_detail.north;
    data.east = msg->nav_detail.east;
    data.down = msg->nav_detail.elevation;

    data_liorf.utm_zone =  msg->nav_detail.zone_char.utmzone_u;
    data_liorf.utm_letter = msg->nav_detail.zone_char.utmzone;
    data_liorf.heading = heading;
    data_liorf.north = msg->nav_detail.north;
    data_liorf.east = msg->nav_detail.east;
    data_liorf.down = msg->nav_detail.elevation;

    veldata.vx = msg->nav_detail.l_velocity_x;
    veldata.vy = msg->nav_detail.l_velocity_y;
    veldata.vz = msg->nav_detail.l_velocity_z;
}
char map_flag = 2;
void mapflagCallback(const std_msgs::Char::ConstPtr& msg) {
    // Fill data with Odometry data
    map_flag = msg->data;
}

ros::Publisher kalman_filtered_data_pub;
ros::Publisher unfiltered_data_pub;

int main(int argc, char **argv) {
    // ROS Init
    ros::init(argc, argv, "udp_sender_node");
    ros::NodeHandle nh;

    // Subscribe to the Odometry topic
    ros::Subscriber odom_sub = nh.subscribe("/localization", 10, odomCallback);

    // Subscribe to the Odometry topic
    ros::Subscriber lio_sub = nh.subscribe("/liorf/mapping/odometry", 10, liorfCallback);

    // Subscribe to the Navigation topic
    ros::Subscriber nav_sub = nh.subscribe("/nav_data", 10, navCallback);

    // Subscribe to the map matching flag
    ros::Subscriber map_flag_sub = nh.subscribe("/map_flag", 10, mapflagCallback);

    // ROS Publisher
    kalman_filtered_data_pub = nh.advertise<geometry_msgs::Point>("kalman_filtered_data", 10);
    unfiltered_data_pub = nh.advertise<geometry_msgs::Point>("unfiltered_data", 10);

    // UDP information
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(12301); // my pc port: 22301 & other pc port: 12301
    serverAddr.sin_addr.s_addr = inet_addr("192.168.4.12"); // IP number

    // UDP socket
    int udpSocket = socket(AF_INET, SOCK_DGRAM, 0);


    // Variable set
    ros::Rate loop_rate(50);  // 50Hz

    int count = 0;
    float init_heading = 0.0;
    float init_east = 0.0;
    float init_north = 0.0;

    float cos_heading = 0.0;
    float sin_heading = 0.0;
    float deg2rad = 3.141592/180.0;
    float rad2deg = 180.0/3.141592;

    //float bias = 4.7581;            // LiDAR & Nav bias
    float bias = -0.4;            // LiDAR & Nav bias
    float init_slam_x = 0.0;
    float init_slam_y = 0.0;
    int mode_change_flag = 0;
    float vec_norm = 100;
    float init_vel_th = 0.3;
    int mode_num = 2;

    while (ros::ok()) {
        // Create a LocationData struct and initialize it
        LocationData udp_data;

        vec_norm = sqrt(veldata.vx*veldata.vx + veldata.vy*veldata.vy + veldata.vz*veldata.vz);
        ////////////////////////// STEP 1. //////////////////////////        

        // init data && map flag matching
        // Map matching succed
        if(map_flag == 1){
            init_heading = (-159.67)*(3.141592/180.0);
            init_east = 321259.48;
            init_north = 4155175.36;

            init_slam_x = 0.0;
            init_slam_y = 0.0;                      

            udp_data.error_flag = 0;        // when slam data initialized, set error_flag to be 0
            //SLAM data-> UTM data > UDP send
            udp_data = generateUdpData(init_heading, init_slam_x, init_slam_y, init_east, init_north, data);

            mode_num = 0;
        }
        // Map matching fail          
        else if (map_flag == 0){
            if(count < 30 && vec_norm < init_vel_th) {
                if(data_liorf.heading != 0.00){
                    // Collect data when 'data_liorf.heading' is not 0.00
                    printf("Where are you\n");
                    init_heading_data.push_back((data_liorf.heading * rad2deg + bias) * (3.141592 / 180.0));
                    init_east_data.push_back(data_liorf.east);
                    init_north_data.push_back(data_liorf.north);
                    init_slam_x_data.push_back(data_liorf.utm_x);
                    init_slam_y_data.push_back(data_liorf.utm_y);

                    printf("Where are you2\n");
                    count++;
                }
            }
            else{
                // Sort collected data in ascending order
                std::sort(init_heading_data.begin(), init_heading_data.end());
                std::sort(init_east_data.begin(), init_east_data.end());
                std::sort(init_north_data.begin(), init_north_data.end());
                std::sort(init_slam_x_data.begin(), init_slam_x_data.end());
                std::sort(init_slam_y_data.begin(), init_slam_y_data.end());

                // Select the middle value as 'init_heading'
                init_heading = init_heading_data[count / 2];
                init_east = init_east_data[count / 2];
                init_north = init_north_data[count / 2];
                init_slam_x = init_slam_x_data[count / 2];
                init_slam_y = init_slam_y_data[count / 2];
                printf("Where are you3\n");

                // Set 'error_flag' to 1 during initialization
                udp_data.error_flag = 1;
            }

            // Reset 'error_flag' to 0 after initialization
            udp_data.error_flag = 0;

            // Generate UDP data
            udp_data = generateUdpData(init_heading, init_slam_x, init_slam_y, init_east, init_north, data_liorf);

            mode_num = 1;
        }
        else{
                init_heading = (0.0)*(3.141592/180.0);
                init_east = data.east;
                init_north = data.north;

                init_slam_x = 0.0;
                init_slam_y = 0.0;

                data.utm_x = 0.0;
                data.utm_y = 0.0;

                // SLAM 동작 실패로, NAV data 그대로 밀어줌
                udp_data.error_flag = 1;

                //SLAM data-> UTM data > UDP send
                udp_data = generateUdpData(init_heading, init_slam_x, init_slam_y, init_east, init_north, data);

                mode_num = 2;
        }

        // Publish Kalman filtered data
/*         geometry_msgs::Point kalman_filtered_msg;
        kalman_filtered_msg.x = (data.east - (udp_data.east + init_east));
        kalman_filtered_msg.y = (data.north - (udp_data.north + init_north));
        kalman_filtered_msg.z = 0.0;
        kalman_filtered_data_pub.publish(kalman_filtered_msg);

        // Publish unfiltered data
        geometry_msgs::Point unfiltered_msg;
        unfiltered_msg.x = (data.east - (val.east + init_east));
        unfiltered_msg.y = (data.north - (val.north + init_north));
        unfiltered_msg.z = 0.0;
        unfiltered_data_pub.publish(unfiltered_msg); */


        geometry_msgs::Point kalman_filtered_msg;
        kalman_filtered_msg.x = data.east;
        kalman_filtered_msg.y = data.north;
        kalman_filtered_msg.z = 0.0;
        kalman_filtered_data_pub.publish(kalman_filtered_msg);

        // Publish unfiltered data
        geometry_msgs::Point unfiltered_msg;
        unfiltered_msg.x = udp_data.east + init_east;
        unfiltered_msg.y = udp_data.north + init_north;
        unfiltered_msg.z = 0.0;
        unfiltered_data_pub.publish(unfiltered_msg);


        ////////////////////////// STEP 4. //////////////////////////
        //////////////////////// Data check /////////////////////////
        /////////////////////////////////////////////////////////////

        // Value check 
        std::cout << " Mode_num        :: " << mode_num << std::endl;
        std::cout << " count           :: " << count << std::endl;
        std::cout << " Init heading    :: " << init_heading*(180/3.141592) << std::endl;
        std::cout << " Heading         :: " << udp_data.heading*(180/3.141592) << std::endl;
        std::cout << std::fixed;
        std::cout.precision(6);
        std::cout << " Udp_data.east   :: " << udp_data.east << std::endl;
        std::cout << " Udp_data.north  :: " << udp_data.north << std::endl;

        std::cout << " UDP Nav east        :: " << (udp_data.east + init_east)<< std::endl;
        std::cout << " UDP Nav north       :: " << (udp_data.north + init_north)<< std::endl;

        std::cout << " REAL Nav east       :: " << (data.east)<< std::endl;
        std::cout << " REAL Nav north      :: " << (data.north)<< std::endl;

        //std::cout << " data.utm_zone   :: " << data.utm_zone << std::endl;
        //std::cout << " data.utm_letter :: " << data.utm_letter << std::endl;

        std::cout << " Nav east error  :: " << (data.east - (udp_data.east + init_east))<< std::endl;
        std::cout << " Nav north error :: " << (data.north - (udp_data.north + init_north))<< std::endl;

        ////////////////////////// STEP 5. //////////////////////////
        ///////////////////////// Data send /////////////////////////
        /////////////////////////////////////////////////////////////
            
        // Send data
        sendto(udpSocket, &udp_data, sizeof(udp_data), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

        std::cout << "========== UDP data send ==========" << std::endl; 
        std::cout << std::endl;

        ////////////////////////// STEP 6. //////////////////////////
        //////////////////////////// etc. ///////////////////////////
        /////////////////////////////////////////////////////////////        

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Close socket
    close(udpSocket);

    std::cout << "UDP socket closed" << std::endl;

    return 0;
}