#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

ros::Publisher imu_pub;
ros::Publisher initialpose_pub;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu modified_imu = *msg; // message copy    

    // y and z axis sign change
    modified_imu.angular_velocity.y = -msg->angular_velocity.y;
    modified_imu.angular_velocity.z = -msg->angular_velocity.z;
    modified_imu.linear_acceleration.y = -msg->linear_acceleration.y;
    modified_imu.linear_acceleration.z = -msg->linear_acceleration.z;

    // quaternion sign change
    modified_imu.orientation.x = -msg->orientation.x;
    modified_imu.orientation.y = -msg->orientation.y;
    modified_imu.orientation.z = -msg->orientation.z;
    modified_imu.orientation.w = msg->orientation.w;

    imu_pub.publish(modified_imu); // publish modified data
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_function_node");
    ros::NodeHandle nh;

    // 토픽 및 메시지 설정
    ros::Publisher initialPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;

    ros::Subscriber imu_sub = nh.subscribe("/imu_data", 10, imuCallback);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_data_slam", 10);


    initialPoseMsg.header.seq = 0;
    initialPoseMsg.header.stamp = ros::Time(0);
    initialPoseMsg.header.frame_id = "";

    initialPoseMsg.pose.pose.position.x = config["position"]["x"].as<double>();
    initialPoseMsg.pose.pose.position.y = config["position"]["y"].as<double>();
    initialPoseMsg.pose.pose.position.z = config["position"]["z"].as<double>();

    initialPoseMsg.pose.pose.orientation.x = config["orientation"]["x"].as<double>();
    initialPoseMsg.pose.pose.orientation.y = config["orientation"]["y"].as<double>();
    initialPoseMsg.pose.pose.orientation.z = config["orientation"]["z"].as<double>();
    initialPoseMsg.pose.pose.orientation.w = config["orientation"]["w"].as<double>();

    for (int i = 0; i < 36; ++i) {
        initialPoseMsg.pose.covariance[i] = 0.0;
    }

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
