#define PCL_NO_PRECOMPILE
#ifndef HEIGHT_MAP_H_
#define HEIGHT_MAP_H_

// ros includes
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
// my includes
#include "height_map/velodyne_type.hpp"
#include "height_map/grid_cell.h"
#include "height_map/grid_map.h"

const int grid_dim = 240;
const double cell_size = 0.05;

namespace height_map
{
typedef grid_map GridPointCloud;

class HeightMap
{
public:
	HeightMap(ros::NodeHandle node, ros::NodeHandle private_node);
	~HeightMap();

private:
    void ProcessData1(const sensor_msgs::PointCloud2ConstPtr& msg);
	void ProcessData2(const sensor_msgs::PointCloud2ConstPtr& msg);
	void ConstructGridMaps(const VPointCloud::ConstPtr& scan,
		                   GridPointCloud& velodyne_map);
	void ConstructAbsoluteMaps(const VPointCloud::ConstPtr& scan,
	 					       GridPointCloud& velodyne_map);
    // void ConstructFullClouds(const VPointCloud::ConstPtr& scan,
  	//      unsigned npoints, size_t& obs_count, size_t& empty_count);
    void ConstructGridClouds(const VPointCloud::ConstPtr& scan,
		 VPointCloud& obstacle_cloud, VPointCloud& clear_cloud,
         unsigned npoints, size_t& obs_count, size_t& empty_count);

	// grid parameters
	int min_points;
	double ground_height;
	double mean_threshold;
	double high_terrain;
	double low_terrain;
	double max_height;
	double diff_terrain;
	// double lidar_height;
	// double lidar_angle;

	// subscribers
	ros::Subscriber velodyne1_scan;
	// publishers
	ros::Publisher map1_publisher;
	ros::Publisher obstacle1_publisher;
	ros::Publisher clear1_publisher;
	ros::Publisher absolute1_publisher;

	// subscribers
	ros::Subscriber velodyne2_scan;
	// publishers
	ros::Publisher map2_publisher;
	ros::Publisher obstacle2_publisher;
	ros::Publisher clear2_publisher;
	ros::Publisher absolute2_publisher;
};
} // namespace height_map
#endif
