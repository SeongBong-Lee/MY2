#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>

struct CustomPoint {
    PCL_ADD_POINT4D // This adds the members x, y, z, intensity (4D)
    PCL_ADD_INTENSITY;
    //float intensity;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    CustomPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (float, time, time)
)

class PointCloudConverter {
private:
    ros::NodeHandle nh_;
    ros::Publisher output_pub_;
    ros::Subscriber input_sub_;

public:
    PointCloudConverter() {
        input_sub_ = nh_.subscribe("/ml_/pointcloud", 1, &PointCloudConverter::inputCallback, this);
        output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/output_pointcloud", 1);
    }

    void inputCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*input_cloud, *cloud);

        pcl::PointCloud<CustomPoint>::Ptr output_cloud(new pcl::PointCloud<CustomPoint>);
        output_cloud->points.resize(cloud->points.size());
        output_cloud->width = cloud->width;
        output_cloud->height = cloud->height;

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            CustomPoint point;
            point.x = cloud->points[i].x;
            point.y = cloud->points[i].y;
            point.z = cloud->points[i].z;

            int32_t rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
            point.intensity = static_cast<float>((rgb) & 0xFFFF);

            // Calculate ring (add your logic here to compute 'ring' information)
            // ... (insert logic to calculate 'ring')
            float depth = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            float pitch = asin(point.z / depth);
            float fov_down = -17.5 * M_PI / 180.0;
            //float fov = (fabs(-17.5) + fabs(35.0)) * M_PI / 180.0;
            float fov = (fabs(-17.5) + fabs(17.5)) * M_PI / 180.0;
            float proj_y = (pitch + fabs(fov_down)) / fov; // in [0.0, 1.0]
            proj_y *= 56; // in [0.0, H]
            proj_y = std::min(56 - 1.0f, proj_y);
            proj_y = std::max(0.0f, proj_y);
            point.ring = static_cast<uint16_t>(proj_y);

            point.time = float(0.0);

            output_cloud->points[i] = point;
        }

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        output_msg.header = input_cloud->header;
        //output_msg.header.frame_id = "base_link";
        output_msg.header.frame_id = "velodyne";

        output_pub_.publish(output_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_function_node");
    PointCloudConverter converter;
    ros::spin();
    return 0;
}