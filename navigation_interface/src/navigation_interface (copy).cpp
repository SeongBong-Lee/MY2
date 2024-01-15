#include "navigation_interface/navigation_interface.h"
#include <tf2/impl/convert.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace navigation_interface {

    Publisher::Publisher()
    {
        port_name = "/dev/ttyS2";
        baud_rate = 115200;

        m_navigation_data_receive_cnt_for_hz_check = 0;
        m_last_navigation_data_receive_second_for_hz_check = 0;
        m_navigation_data_receive_hz = 0;

        timer_ = nh_.createTimer(ros::Duration(0.02), &Publisher::timerCallback, this, false, false);
	    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_data", 1000);
               
        nav_pub_ = nh_.advertise<interfaces_msgs::AutoNavigation>("nav_data", 100);
        nav_status_pub_ = nh_.advertise<interfaces_msgs::NavigationSystemStatus>("nav_status_data", 100);

        serial_open(port_name, baud_rate);
        timer_.start();

    }

    Publisher::~Publisher()
    {
        delete [] p_serial_data_buffer;
        gps_serial.close();
        timer_.stop();

    }

    void Publisher::timerCallback(const ros::TimerEvent&)
    {
        read_and_parsing();

    }

    unsigned short Publisher::calculate_crc16(const void * data, unsigned short length)
    {
        unsigned char *bytes = (unsigned char *)data;
        unsigned short crc = 0x0000, i;
        for (i = 0; i < length; i++)
        {
            crc = (unsigned short)((crc << 8) ^ crc16_CCITT_table[(crc >> 8) ^ bytes[i]]);
        }
        return crc;
    }
    void Publisher::publish_imu(GPS_INS_NAV nav_stuct){

        sensor_msgs::Imu imu_data;
        geometry_msgs::Quaternion quat;
        tf2::Quaternion quat_tf;

        double roll = (float)nav_stuct.Attitude.att_roll * 0.1 * 0.0009817477; //float64();
        double pitch = (float)nav_stuct.Attitude.att_pitch * 0.1 * 0.0009817477; //float64();
        double yaw = (float)nav_stuct.Attitude.att_yaw * 0.1 * 0.0009817477; //float64();
        
        quat_tf.setRPY(roll,pitch,yaw);
        quat_tf.normalize();

        tf2::convert(quat_tf, quat);

        imu_data.orientation = quat;

        imu_data.angular_velocity.x = (float)nav_stuct.Gyroscope_X * 0.001; 
        imu_data.angular_velocity.y = (float)nav_stuct.Gyroscope_Y * 0.001;
        imu_data.angular_velocity.z = (float)nav_stuct.Gyroscope_Z * 0.001;

        imu_data.linear_acceleration.x = (float)nav_stuct.Accelerometer_X * 0.01; 
        imu_data.linear_acceleration.y = (float)nav_stuct.Accelerometer_Y * 0.01;
        imu_data.linear_acceleration.z = (float)nav_stuct.Accelerometer_Z * 0.01;
        /*
        float ori_cov[9] = [nav_stuct.Covariance_Attitude_Roll, nav_stuct.Covariance_Attitude_Pitch, nav_stuct.Covariance_Attitude_Yaw];
        float ang_cov[9] = [nav_stuct.Covariance_Accel_X, nav_stuct.Covariance_Accel_Y, nav_stuct.Covariance_Accel_Z];
        float acc_cov[9] = [nav_stuct.Covariance_Gyro_X, nav_stuct.Covariance_Gyro_X, nav_stuct.Covariance_Gyro_X];
        
        imu_data.orientation_covariance = ori_cov;
        imu_data.angular_velocity_covariance = ang_cov;
        imu_data.linear_acceleration_covariance = acc_cov;
        */
        imu_pub_.publish(imu_data);	

    }
    void Publisher::publish_nav(GPS_INS_NAV nav_stuct){
        
        interfaces_msgs::AutoNavigation pub_nav_data;
        interfaces_msgs::NavigationDetail nav_data;
        interfaces_msgs::ZoneChar zone_data;

        nav_data.timestamp = ros::Time::now().toSec(); //float64();

        zone_data.utmzone_u = nav_stuct.zone_number; // uint8_t();
        zone_data.utmzone = nav_stuct.zone_letter; //int8_t();

        nav_data.zone_char = zone_data; //ZoneChar();
        nav_data.east = (float)nav_stuct.east * 0.01; //float64(); m
        nav_data.north = (float)nav_stuct.north * 0.01; //float64();
        nav_data.elevation = (float)nav_stuct.height * 0.01; //float64();
        nav_data.roll = (float)nav_stuct.Attitude.att_roll * 0.1; //float64();
        nav_data.pitch = (float)nav_stuct.Attitude.att_pitch * 0.1; //float64();
        nav_data.heading = (float)nav_stuct.Attitude.att_yaw * 0.1; //float64();

        nav_data.l_velocity_x = (float)nav_stuct.body_vel.vel_body_x * 0.0001; //float64();
        nav_data.l_velocity_y = (float)nav_stuct.body_vel.vel_body_y * 0.0001; //float64();
        nav_data.l_velocity_z = (float)nav_stuct.body_vel.vel_body_z * 0.0001; //float64();
        nav_data.a_velocity_x = (float)nav_stuct.Gyroscope_X * 0.001; //float64();
        nav_data.a_velocity_y = (float)nav_stuct.Gyroscope_Y * 0.001; //float64();
        nav_data.a_velocity_z = (float)nav_stuct.Gyroscope_Z * 0.001; //float64();

        pub_nav_data.synctime = nav_data.timestamp;
        pub_nav_data.nav_detail = nav_data;
        
        nav_pub_.publish(pub_nav_data);
        
    }

    void Publisher::publish_nav_status(GPS_INS_NAV nav_stuct){
        
        interfaces_msgs::NavigationSystemStatus pub_nav_data;
        interfaces_msgs::NavigationStatus nav_data;
        interfaces_msgs::GpsStatus gps_data;
        interfaces_msgs::SatelliteStatus satellite_data;

        nav_data.dual_ant_adding = nav_stuct.NAV_status.Reserved;
        nav_data.zupt = nav_stuct.NAV_status.ZUPT;
        nav_data.vms_adding = nav_stuct.NAV_status.VMS_add;

        if(nav_stuct.NAV_status.GPS_INS)
        {
            nav_data.sensing_mode = 3;
        }
        else if(nav_stuct.NAV_status.INS_only)
        {
            nav_data.sensing_mode = 2;

        }
        else if(nav_stuct.NAV_status.GPS_only)
        {
            nav_data.sensing_mode = 1;

        }
        else
        {
            nav_data.sensing_mode = 0;
        }

        nav_data.init_align = nav_stuct.NAV_status.allign;
        nav_data.unabled = nav_stuct.NAV_status.invaild;

        gps_data.hot_start = nav_stuct.GPS_status.HOT;
        gps_data.warm_start = nav_stuct.GPS_status.WARM;
        gps_data.cold_start = nav_stuct.GPS_status.COLD;
        
        gps_data.nav_invalid = nav_stuct.GPS_status.nav_invaild;
        gps_data.reaquisition = nav_stuct.GPS_status.reacq;
        gps_data.rtk = nav_stuct.GPS_status.RTK;

        if(nav_stuct.GPS_status.nav_2D)
        {
            gps_data.nav_mode = 2;
        }
        else if(nav_stuct.GPS_status.nav_3D)
        {
            gps_data.nav_mode = 1;
        }
        else
        {
            gps_data.nav_mode = 0;
        }

        satellite_data.num_satellite = nav_stuct.tracking_sv;
        satellite_data.pdop = nav_stuct.sat_dop.pdop * 0.1;
        satellite_data.hdop = nav_stuct.sat_dop.hdop * 0.1;
        satellite_data.vdop = nav_stuct.sat_dop.vdop * 0.1;

        pub_nav_data.timestamp = ros::Time::now().toSec();
        pub_nav_data.nav_status = nav_data;
        pub_nav_data.gps_status = gps_data;
        pub_nav_data.satellite_status = satellite_data;

        nav_status_pub_.publish(pub_nav_data);
        
    }

    void Publisher::serial_open(std::string portname, int baudrate)
    {

        gps_serial.setPort(portname);
        gps_serial.setBaudrate(baudrate);
        gps_serial.setBytesize(serial::bytesize_t::eightbits);
        gps_serial.setParity(serial::parity_t::parity_none);
        gps_serial.setStopbits(serial::stopbits_t::stopbits_two);
        gps_serial.setFlowcontrol(serial::flowcontrol_t::flowcontrol_none);

        gps_serial.open();
        
        if(!gps_serial.isOpen()){
            ROS_ERROR("[NavigationInterface] couldn't open serial port");
            ros::shutdown();
        }
    }

    // 1����Ʈ ��Ʈ���� Endian ��ȯ
    void Publisher::convert_endian_bit(unsigned char* p_data)
    {
        unsigned char temp = 0x00;
        temp = (unsigned char)(((*p_data >> 7) & 0x01) << 0) +
            (unsigned char)(((*p_data >> 6) & 0x01) << 1) +
            (unsigned char)(((*p_data >> 5) & 0x01) << 2) +
            (unsigned char)(((*p_data >> 4) & 0x01) << 3) +
            (unsigned char)(((*p_data >> 3) & 0x01) << 4) +
            (unsigned char)(((*p_data >> 2) & 0x01) << 5) +
            (unsigned char)(((*p_data >> 1) & 0x01) << 6) +
            (unsigned char)(((*p_data >> 0) & 0x01) << 7);
        *p_data = temp;
    }


    // 2����Ʈ Endian ��ȯ
    void Publisher::convert_endian_2byte(char* p_data)
    {
        char temp_data[2];
        memset(temp_data, 0, 2);
        temp_data[1] = p_data[0];
        temp_data[0] = p_data[1];
        memcpy(p_data, temp_data, 2);
    }

    // 4����Ʈ Endian ��ȯ
    void Publisher::convert_endian_4byte(char* p_data)
    {
        char temp_data[4];
        temp_data[3] = p_data[0];
        temp_data[2] = p_data[1];
        temp_data[1] = p_data[2];
        temp_data[0] = p_data[3];
        memcpy(p_data, temp_data, 4);
    }

    // 8����Ʈ Endian ��ȯ
    void Publisher::convert_endian_8byte(char* p_data)
    {
        char temp_data[8];
        temp_data[7] = p_data[0];
        temp_data[6] = p_data[1];
        temp_data[5] = p_data[2];
        temp_data[4] = p_data[3];
        temp_data[3] = p_data[4];
        temp_data[2] = p_data[5];
        temp_data[1] = p_data[6];
        temp_data[0] = p_data[7];
        memcpy(p_data, temp_data, 8);
    }

    void Publisher::convert_navigation_struct_endian(GPS_INS_NAV* p_data)
    {
        GPS_INS_NAV temp_data;
        memcpy(&temp_data, p_data, sizeof(GPS_INS_NAV));

        convert_endian_2byte((char*)&temp_data.UTC_TIME.msec);
        convert_endian_2byte((char*)&temp_data.gps_week);
        convert_endian_4byte((char*)&temp_data.gps_tow);
        convert_endian_4byte((char*)&temp_data.lat);
        convert_endian_4byte((char*)&temp_data.lon);
        convert_endian_4byte((char*)&temp_data.east);
        convert_endian_4byte((char*)&temp_data.north);

        convert_endian_4byte((char*)&temp_data.height);

        convert_endian_2byte((char*)&temp_data.Attitude.att_roll);
        convert_endian_2byte((char*)&temp_data.Attitude.att_pitch);
        convert_endian_2byte((char*)&temp_data.Attitude.att_yaw);

        convert_endian_4byte((char*)&temp_data.body_vel.vel_body_x);
        convert_endian_4byte((char*)&temp_data.body_vel.vel_body_y);
        convert_endian_4byte((char*)&temp_data.body_vel.vel_body_z);
        convert_endian_2byte((char*)&temp_data.body_vel.vel_body_magnitude);

        convert_endian_bit((unsigned char*)&temp_data.NAV_status);
        convert_endian_bit((unsigned char*)&temp_data.GPS_status);

        convert_endian_2byte((char*)&temp_data.Accelerometer_X);
        convert_endian_2byte((char*)&temp_data.Accelerometer_Y);
        convert_endian_2byte((char*)&temp_data.Accelerometer_Z);
        convert_endian_2byte((char*)&temp_data.Gyroscope_X);
        convert_endian_2byte((char*)&temp_data.Gyroscope_Y);
        convert_endian_2byte((char*)&temp_data.Gyroscope_Z);
        /*
        convert_endian_8byte((char*)&temp_data.Covariance_Attitude_Roll);
        convert_endian_8byte((char*)&temp_data.Covariance_Attitude_Pitch);
        convert_endian_8byte((char*)&temp_data.Covariance_Attitude_Yaw);
        convert_endian_8byte((char*)&temp_data.Covariance_Accel_X);
        convert_endian_8byte((char*)&temp_data.Covariance_Accel_Y);
        convert_endian_8byte((char*)&temp_data.Covariance_Accel_Z);
        convert_endian_8byte((char*)&temp_data.Covariance_Gyro_X);
        convert_endian_8byte((char*)&temp_data.Covariance_Gyro_Y);
        convert_endian_8byte((char*)&temp_data.Covariance_Gyro_Z);
        //check cov
        */
        memcpy(p_data, &temp_data, sizeof(GPS_INS_NAV));
    }

    void Publisher::read_and_parsing()
    {

        //while(ros::ok){

            int read_size = gps_serial.read(p_serial_data_buffer, serial_data_buffer_size);
            
            // �ø��� ������ �о ���ۿ� ����
            if(serial_data_buffer_token + read_size < serial_data_buffer_size)
            {
                memcpy(p_total_serial_data_buffer + serial_data_buffer_token, p_serial_data_buffer, read_size);

                serial_data_buffer_token += read_size;
            }

            if(serial_data_buffer_token < 2)
            {     
                ROS_ERROR("[NavigationInterface] no data");
            }
            ROS_ERROR("[NavigationInterface] buff : %d, read : %d ", serial_data_buffer_token, read_size);


            // // ������ ��� ���߱�
            if(p_total_serial_data_buffer[0] != 0x81 || p_total_serial_data_buffer[1] != 0x81)
            {
                ROS_ERROR("[NavigationInterface] hearder matching");

                int enable_start_array = -1;
                for(int chk_arr = 1 ; chk_arr < serial_data_buffer_token -2 && enable_start_array==-1; chk_arr++)
                {
                    if(p_total_serial_data_buffer[chk_arr] == 0x81 || p_total_serial_data_buffer[chk_arr+1] == 0x81)
                    {
                        enable_start_array = chk_arr;
                    }
                }

                if(enable_start_array>=0 && enable_start_array<serial_data_buffer_token)
                {
                    int cpy_size = serial_data_buffer_token - enable_start_array;
                    memcpy(p_total_serial_data_buffer, p_total_serial_data_buffer+enable_start_array, cpy_size);
                    serial_data_buffer_token -= enable_start_array;
                }
                 
            }

            struct_serial_data_packet receive_packet;

            // // ������ �Ľ�
            if(p_total_serial_data_buffer[0] == 0x81 || p_total_serial_data_buffer[1] == 0x81)
            {
                ROS_ERROR("[NavigationInterface] hearder parshing");

                if(serial_data_buffer_token>6)
                {

                    int msg_id = p_total_serial_data_buffer[3];
                    int msg_len = p_total_serial_data_buffer[4];
                    int msg_full_size = 10 + msg_len;

                    memcpy(&receive_packet, p_total_serial_data_buffer, 6);
                    
                    int packet_size = sizeof(struct_serial_data_packet) - sizeof(unsigned char*) + receive_packet.length;
                    
                    if( serial_data_buffer_token >= packet_size )
                    {
                        ROS_ERROR("[NavigationInterface] packet_size check");
                        ROS_ERROR("[NavigationInterface] receive_msg_id : %d",  receive_packet.msg_ID);
                        ROS_ERROR("[NavigationInterface] receive_len : %d",  msg_len);

                        if(receive_packet.msg_ID == 0x56)
                        {
                            ROS_ERROR("[NavigationInterface] valid msg id");

                            int data_size = receive_packet.length;
                            if(p_total_serial_data_buffer[8 + data_size] == 0x0D && p_total_serial_data_buffer[8 + data_size + 1] == 0x0A )
                            {
                                bool enable_crc_value_flag = true;
                                unsigned short receive_big_endiqn_crc = 0x00;
                                memcpy(&receive_big_endiqn_crc, p_total_serial_data_buffer + sizeof(struct_serial_data_packet) + receive_packet.length - sizeof(unsigned char*) - 4, 2);
                                convert_endian_2byte((char*)&receive_big_endiqn_crc);
                                unsigned short requirement_CRC = calculate_crc16(p_total_serial_data_buffer + 2, receive_packet.length + 4);   // address�ʵ���� Data ������ �ʵ���� 2 byte CRC �ڵ�

                                // ��Ŷ ����� ��ȿ���� ���� ���, �ܿ� ������ ���� ���
                                if (receive_big_endiqn_crc != requirement_CRC)
                                {
                                    enable_crc_value_flag = false;
                                    int remain_size = serial_data_buffer_token - packet_size;
                                    memcpy(p_total_serial_data_buffer, p_total_serial_data_buffer + packet_size, remain_size);
                                    serial_data_buffer_token -= remain_size;
                                }

                                // ��Ŷ ����� ��ȿ�� ��쿡�� ó��
                                if (enable_crc_value_flag == true)
                                {
                                    // ������ �Ľ� ����
                                    memcpy(&m_redeive_GPS_INS_Nav_data, p_total_serial_data_buffer + 6, receive_packet.length);
                                    convert_navigation_struct_endian(&m_redeive_GPS_INS_Nav_data);
                                    ROS_ERROR("[NavigationInterface] print");

                                    fprintf(p_file, "time=%d : \n", m_navigation_data_receive_cnt_for_hz_check);                                    
                                    fprintf(p_file, "gps_week : %d, gps_tow %d, \n lat: %5.f deg, lon : %5.f deg, \n  east : %5.f m, north : %5.f m \n", m_redeive_GPS_INS_Nav_data.gps_week, m_redeive_GPS_INS_Nav_data.gps_tow, float(m_redeive_GPS_INS_Nav_data.lat) * 0.0000001, float(m_redeive_GPS_INS_Nav_data.lon) * 0.0000001, m_redeive_GPS_INS_Nav_data.east *0.01, m_redeive_GPS_INS_Nav_data.north *0.01);
                                    fprintf(p_file, "\n");

                                    //mp_obs->on_receive_Navigation(m_redeive_GPS_INS_Nav_data, m_navigation_data_receive_hz, none_display_flag);
                                    
                                    publish_nav(m_redeive_GPS_INS_Nav_data);
                                    publish_nav_status(m_redeive_GPS_INS_Nav_data);
                                    publish_imu(m_redeive_GPS_INS_Nav_data);

                                    ros::Time current_time = ros::Time::now();
                                    int current_second = (int)current_time.toSec();
                                    if(current_second!=m_last_navigation_data_receive_second_for_hz_check)
                                    {
                                        //printf("Navigation Data receive Hz = %d\n", m_navigation_data_receive_cnt_for_hz_check);
                                        m_navigation_data_receive_hz = m_navigation_data_receive_cnt_for_hz_check;
                                        m_navigation_data_receive_cnt_for_hz_check = 0;
                                        m_last_navigation_data_receive_second_for_hz_check = current_second;
                                    }
                                    else
                                    {
                                        m_navigation_data_receive_cnt_for_hz_check++;
                                    }
                                }


                            }
                        }
                        else
                        {
                            /*
                            ROS_ERROR("[NavigationInterface] invalid msg id");

                            int remain_size = serial_data_buffer_token - msg_full_size;
                            memcpy(p_total_serial_data_buffer, p_total_serial_data_buffer + msg_full_size, remain_size);
                            serial_data_buffer_token -= remain_size;
                            */

                        }
                                        
                        if(serial_data_buffer_token>=msg_full_size)
                        {
                            int cpy_size = serial_data_buffer_token - packet_size;
                            memcpy(p_total_serial_data_buffer, p_total_serial_data_buffer + msg_full_size, cpy_size);
                            serial_data_buffer_token -= packet_size;
                            ROS_ERROR("[NavigationInterface] remove");

                        }

                    }
                }
            }

        //}
        //gps_serial.close();


        //exit(0);
    }
}
