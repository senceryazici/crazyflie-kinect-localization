#include <ros/ros.h>
#include <string>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //Create point cloud objects for downsampled_cloud and passed_cloud
    pcl::PCLPointCloud2* downsampled_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(downsampled_cloud);
    pcl::PCLPointCloud2 passed_cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *downsampled_cloud);

    // Perform the PassThrough Filter
    pcl::PassThrough<pcl::PCLPointCloud2> p_filter;

    p_filter.setInputCloud(cloudPtr); // Pass downsampled_cloud to the filter
    p_filter.setFilterFieldName("x"); // Set axis
    p_filter.setFilterLimits(0, 2); // Set limits min_value to max_value
    p_filter.filter(passed_cloud); // Store output data in passed_cloud

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(passed_cloud, output_cloud);

    // pub.publish(output_cloud); // Publish passed_cloud to the /cloud_passed topic
}


int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "passthrough_filter");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");

    // Create Subscriber and listen /cloud_downsampled topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/kinect2/qhd/points", 1, callback);

    // Create Publisher
    // pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
