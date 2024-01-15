#ifndef NAVIGATION_INTERFACE_H
#define NAVIGATION_INTERFACE_H


#include <memory>

#include <ros/ros.h>
#include "serial/serial.h"
#include <tf2/LinearMath/Quaternion.h>


#include <iostream>
#include <vector>
#include <unistd.h>
#include <sensor_msgs/Imu.h>

#include "struct_EGIS_N400.h"
#include "const_crc16_table.h"

#include <interfaces_msgs/NavigationDetail.h>
#include <interfaces_msgs/ZoneChar.h>
#include <interfaces_msgs/AutoNavigation.h>
#include <interfaces_msgs/NavigationStatus.h>
#include <interfaces_msgs/NavigationSystemStatus.h>
#include <interfaces_msgs/GpsStatus.h>
#include <interfaces_msgs/SatelliteStatus.h>

using std::placeholders::_1;

namespace navigation_interface {

    class Publisher
    {
        public:

            Publisher();
            ~Publisher();

            unsigned short calculate_crc16(const void * data, unsigned short length);
            void serial_open(std::string portname, int baudrate);
            void convert_endian_bit(unsigned char* p_data);                                             // 1����Ʈ ��Ʈ���� Endian ��ȯ
            void convert_endian_2byte(char* p_data);                                                    // 2����Ʈ Endian ��ȯ
            void convert_endian_4byte(char* p_data);                                                    // 4����Ʈ Endian ��ȯ
            void convert_endian_8byte(char* p_data);                
            void convert_navigation_struct_endian(GPS_INS_NAV* p_data);
            void publish_nav(GPS_INS_NAV nav_stuct);
            void publish_nav_status(GPS_INS_NAV nav_stuct);
            void timerCallback(const ros::TimerEvent&);
            void read_and_parsing();
    	    void publish_imu(GPS_INS_NAV nav_stuct);
            ///////ros////////////////////

            ros::NodeHandle nh_;

            ros::Publisher nav_status_pub_;  // To publish navigation status.
            ros::Publisher nav_pub_;  // To publish navigation status.
            ros::Publisher imu_pub_;  // To publish navigation status.
            ros::Timer timer_; // To publish at specific period

            ///////init////////////////////

            std::string input_topic;
            std::string output_topic;

            std::string port_name;
            unsigned int baud_rate;

            ///////////////////////////////

            unsigned int serial_data_buffer_size = 1024*1024*10;
            unsigned char* p_total_serial_data_buffer = new unsigned char[serial_data_buffer_size];  // 10MB ũ���� ����
            unsigned char* p_serial_data_buffer = new unsigned char[serial_data_buffer_size];  // 10MB ũ���� ����
            unsigned char* p_clear_buffer = new unsigned char[serial_data_buffer_size];  // 10MB ũ���� ����

            unsigned int serial_data_buffer_token = 0;

            FILE* p_file = fopen("/home/hdrt/rs422_ros.txt","wt");

            //////////////////////////////

            unsigned char* mp_buffer;
            int m_serial_data_grab_array;

            std::vector<char> data_array;  

            int m_navigation_data_receive_cnt_for_hz_check;
            int m_last_navigation_data_receive_second_for_hz_check;
            int m_navigation_data_receive_hz;

            GPS_INS_NAV m_redeive_GPS_INS_Nav_data;

            serial::Serial gps_serial;

    };

}

#endif //NAVIGATION_INTERFACE_H
