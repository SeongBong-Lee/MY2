#include "ros/ros.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

ros::Publisher test_pub;
ros::NodeHandle *nh;

void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    // 받은 nav_msgs/Path 메시지에서 가장 최근의 위치 정보 추출하여 nav_msgs/Odometry로 변환
    if (!path_msg->poses.empty()) {
        nav_msgs::Odometry odom_msg;

        // nav_msgs/Path의 마지막 위치 정보를 가져와 nav_msgs/Odometry에 설정
        geometry_msgs::PoseStamped latest_pose = path_msg->poses.back();
        odom_msg.pose.pose.position = latest_pose.pose.position;

        // 새로운 메시지를 publish
        test_pub.publish(odom_msg);
    } else {
        ROS_WARN("Received empty path message.");
        return;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_function_node");
    nh = new ros::NodeHandle();

    test_pub = nh->advertise<nav_msgs::Odometry>("/test", 10);

    ros::Subscriber sub = nh->subscribe("/liorf/mapping/path", 10, pathCallback);

    ros::Rate loop_rate(50);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete nh;
    return 0;
}