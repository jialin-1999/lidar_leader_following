#define PCL_NO_PRECOMPILE
// std includes
#include <time.h>
// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// my includes
#include <lidar_calibration/velodyne_type.hpp>
#include <lidar_calibration/velodyne_calibration.hpp>
// dynamic configure
#include <dynamic_reconfigure/server.h>
#include <lidar_calibration/lidar_calibrationConfig.h>

using std::cout;
using std::endl;

PointCloudXYZIR::Ptr cloud1(new PointCloudXYZIR), cloud2(new PointCloudXYZIR),
                     cloud1_range(new PointCloudXYZIR), cloud2_range(new PointCloudXYZIR),
                     cloud1_calibrated(new PointCloudXYZIR), cloud2_calibrated(new PointCloudXYZIR);

ros::Publisher pub1, pub2;

float lidar1_range, lidar1_height,lidar1_pitch, lidar1_roll, lidar1_yaw,
      lidar2_range, lidar2_height,lidar2_pitch, lidar2_roll, lidar2_yaw;

void ConfigCb(lidar_calibration::lidar_calibrationConfig &config, uint32_t level)
{
    lidar1_range = config.range1;
    lidar1_height = config.height1;
    lidar1_pitch = config.pitch1;
    lidar1_roll = config.roll1;
    lidar1_yaw = config.yaw1;

    lidar2_range = config.range2;
    lidar2_height = config.height2;
    lidar2_pitch = config.pitch2;
    lidar2_roll = config.roll2;
    lidar2_yaw = config.yaw2;
}

void velodyne1_callback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    clock_t start, end;
    start = clock();

    // Convert to PCL data type
    pcl::fromROSMsg(*msg,*cloud1);
    // range cut off
    velodyne_range(cloud1, cloud1_range, lidar1_range);
    // Perform calibration
    velodyne_calibration(cloud1_range, cloud1_calibrated, lidar1_height,
                         lidar1_pitch, lidar1_roll, lidar1_yaw);
    // Convert to ROS data type
    sensor_msgs::PointCloud2Ptr msg_calibrated(new sensor_msgs::PointCloud2);

    pcl::toROSMsg(*cloud1_calibrated,*msg_calibrated);
    msg_calibrated->header.frame_id = std::string("velodyne");
    msg_calibrated->header.stamp = ros::Time::now();
    msg_calibrated->header.seq = msg->header.seq;
    // Publish the data
    pub1.publish (msg_calibrated);

    end = clock();
    //cout << (end - start) * 1.0 / CLOCKS_PER_SEC * 1000 << " ms, Lidar_calibration time" << endl;
}

void velodyne2_callback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    clock_t start, end;
    start = clock();

    // Convert to PCL data type
    pcl::fromROSMsg(*msg,*cloud2);
    // range cut off
    velodyne_range(cloud2, cloud2_range, lidar2_range);
    // Perform calibration
    velodyne_calibration(cloud2_range, cloud2_calibrated, lidar2_height,
                         lidar2_pitch, lidar2_roll, lidar2_yaw);
    // Convert to ROS data type
    sensor_msgs::PointCloud2Ptr msg_calibrated(new sensor_msgs::PointCloud2);

    pcl::toROSMsg(*cloud2_calibrated,*msg_calibrated);
    msg_calibrated->header.frame_id = std::string("velodyne");
    msg_calibrated->header.stamp = ros::Time::now();
    msg_calibrated->header.seq = msg->header.seq;
    // Publish the data
    pub2.publish (msg_calibrated);

    end = clock();
    //cout << (end - start) * 1.0 / CLOCKS_PER_SEC * 1000 << " ms, Lidar_calibration time" << endl;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "lidar_calibration");
    ros::NodeHandle nh;

    // dynamic_reconfigure
    dynamic_reconfigure::Server<lidar_calibration::lidar_calibrationConfig> server;
    dynamic_reconfigure::Server<lidar_calibration::lidar_calibrationConfig>::CallbackType f;

    f = boost::bind(&ConfigCb, _1, _2);
    server.setCallback(f);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2> ("ns1/velodyne_points", 1, velodyne1_callback);
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2> ("ns2/velodyne_points", 1, velodyne2_callback);
    // Create a ROS publisher for the output point cloud
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("velodyne1_calibration", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2> ("velodyne2_calibration", 1);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
}
