#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudProcessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber ml_pointcloud_sub_;
    ros::Subscriber ouster_points_sub_;
    ros::Publisher output_pointcloud_pub_;

public:
    PointCloudProcessor() : nh_("~") {
        // Subscribe to /ml_/pointcloud
        ml_pointcloud_sub_ = nh_.subscribe("/ml_/pointcloud", 1,
                                           &PointCloudProcessor::mlPointCloudCallback, this);

        // Subscribe to /ouster/points
        ouster_points_sub_ = nh_.subscribe("/ouster/points", 1,
                                           &PointCloudProcessor::ousterPointsCallback, this);

        // Publish to /output_pointcloud
        output_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/output_pointcloud", 1);
    }

    void mlPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ml_pointcloud) {
        // Publish ml_/pointcloud data to output_pointcloud
        output_pointcloud_pub_.publish(ml_pointcloud);
    }

    void ousterPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& ouster_points) {
        // Modify ouster_points to update timestamp only and publish to output_pointcloud
        sensor_msgs::PointCloud2 modified_points = *ouster_points;
        modified_points.header.stamp = ros::Time::now(); // Update timestamp

        output_pointcloud_pub_.publish(modified_points);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_function_node");

    PointCloudProcessor processor;

    ros::spin();

    return 0;
}
