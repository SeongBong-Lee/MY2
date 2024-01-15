
#pragma once

namespace  struct_EGIS_N400
{ 
    #pragma pack(1)

	// Serial data Packet
	struct struct_serial_data_packet
	{
		unsigned short start;
		unsigned char address;
		unsigned char msg_ID;
		unsigned char length;
		unsigned char status;
		unsigned char* p_data;
		unsigned short CRC_16;
		unsigned short end_flag;
	};


	struct struct_UTC_TIME
	{
		unsigned char year;
		unsigned char month;
		unsigned char day;
		unsigned char hour;
		unsigned char minute;
		unsigned char sec;
		unsigned short msec;
	};

	struct struct_ATT
	{
		short att_roll;									//�ڼ� Roll (mil)
		short att_pitch;								//�ڼ� Pitch (mil)
		short att_yaw;									//�ڼ� Yaw (mil)
	};

	struct struct_body_vel
	{
		int vel_body_x;								//��ü�ӵ� X(m/s)
		int vel_body_y;								//��ü�ӵ� Y(m/s)
		int vel_body_z;								//��ü�ӵ� Z(m/s)
		unsigned int vel_body_magnitude;						//��ü�ӵ� Magnitude(m/s)
	};
	
	struct struct_NAV_status
	{
		unsigned char Reserved : 1;                     // Reserved
        unsigned char ZUPT : 1;                         // ZUPT
        unsigned char VMS_add : 1;                      // VMS Adding
        unsigned char GPS_INS : 1;                      // GPS + INS
        unsigned char GPS_only : 1;                     // GPS only
        unsigned char INS_only : 1;                     // INS only
        unsigned char allign : 1;                       // �ʱ�����
        unsigned char invaild : 1;                      // �׹��Ұ�
	};

	struct struct_GPS_status
	{
		unsigned char COLD : 1;							 // COLD start
		unsigned char WARM : 1;							 // WARM start
		unsigned char HOT : 1;							 // HOT start
		unsigned char reacq : 1;						 // Reacqusition
		unsigned char nav_invaild : 1;                   // Navigation Invaild
		unsigned char nav_2D : 1;						 // 2D Navigation
		unsigned char nav_3D : 1;                        // 3D Navigation
		unsigned char RTK : 1;                      // Reserved
	};

	struct struct_Sat_DOP
	{
		unsigned char reserved;
		unsigned char pdop;									// GPS PDOP   scale : 0.1
		unsigned char hdop;									// GPS HDOP   scale : 0.1
		unsigned char vdop;									// GPS VDOP   scale : 0.1
	};

	
	struct GPS_INS_NAV
	{
		struct_UTC_TIME UTC_TIME;
		unsigned short gps_week;						//GPS Week Number
		unsigned int gps_tow;							//GPS Time on Week
		int lat;										//Latitude (degree)  // scale : e-7
		int lon;										//Longitude (degree)
		unsigned int east;								//���� (meter)
		unsigned int north;								//�ϰ� (meter)
		unsigned char zone_number;						//�����ȣ UTM ���� ����1~60
		unsigned char zone_letter;						//�����ȣ UTM ���� ����'C'~'X'
		int height;										//���� (meter)
		struct_ATT Attitude;
		struct_body_vel body_vel;
		struct_NAV_status NAV_status;
		struct_GPS_status GPS_status;
		unsigned char Reserved;						
		unsigned char tracking_sv;						// GPS ������������
		struct_Sat_DOP sat_dop;
		short		Accelerometer_X;					//! Raw Sensor Data, scale 0.01
		short		Accelerometer_Y;					//! Raw Sensor Data, scale 0.01
		short		Accelerometer_Z;					//! Raw Sensor Data, scale 0.01
		short		Gyroscope_X;						//! Raw Sensor Data
		short		Gyroscope_Y;						//! Raw Sensor Data
		short		Gyroscope_Z;						//! Raw Sensor Data
		/*
		double		Covariance_Attitude_Roll;			// radian
		double		Covariance_Attitude_Pitch;			// radian
		double		Covariance_Attitude_Yaw;			// radian
		double		Covariance_Accel_X;					// m/s2
		double		Covariance_Accel_Y;					// m/s2
		double		Covariance_Accel_Z;					// m/s2
		double		Covariance_Gyro_X;					// rad/s
		double		Covariance_Gyro_Y;					// rad/s
		double		Covariance_Gyro_Z;					// rad/s
		*/
	};


    #pragma pack()

}

using namespace struct_EGIS_N400;
