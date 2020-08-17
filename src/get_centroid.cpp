// Import dependencies
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/crop_box.h>
#include <geometry_msgs/PointStamped.h>

// Definitions
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    //Create point cloud objects for downsampled_cloud and passed_cloud
    pcl::PCLPointCloud2* downsampled_cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*cloud_msg, *downsampled_cloud);

    pcl::PointXYZRGB centroid;
    pcl::fromPCLPointCloud2(*downsampled_cloud, *temp_cloud);
    pcl::computeCentroid(*temp_cloud, centroid);

    geometry_msgs::PointStamped msg;
    msg.header = cloud_msg->header;
    msg.point.x = centroid.x;
    msg.point.y = centroid.y;
    msg.point.z = centroid.z;

    pub.publish(msg);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "get_centroid");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");

    // Create Subscriber and listen /cloud_downsampled topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("points_in", 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<geometry_msgs::PointStamped>("centroid", 1);

    // Spin
    ros::spin();
    return 0;
}
