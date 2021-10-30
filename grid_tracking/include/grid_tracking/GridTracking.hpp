#ifndef GRID_TRACKING_H_
#define GRID_TRACKING_H_
// std includes
#include <iostream>
#include <vector>
#include <string>
// ros includes
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen3/Eigen/Core>
#include <std_msgs/Float64MultiArray.h>
// msg includes
#include "height_map/grid_cell.h"
#include "height_map/grid_map.h"
#include "grid_tracking/cluster_map.h"
#include "grid_tracking/grid_clusters.h"
#include "grid_tracking/grid_cells.h"
// my includes
#include "grid_tracking/velodyne_type.hpp"
#include "grid_tracking/grid_size.hpp"

using Eigen::Vector3f;
using std::vector;
using std::pair;

typedef height_map::grid_map GridPointCloud;
typedef grid_tracking::cluster_map TerrainMap;
typedef grid_tracking::grid_clusters TerrainClusters;
typedef grid_tracking::grid_cells TerrainCells;

struct TargetInfo
{
    bool target;
    VPoint pos;
    PointXYZ sz;
    size_t count;
    vector<double> histogram;
    VPointCloud cloud;

    bool operator < (const TargetInfo& right) const
    {
        return pos.intensity < right.pos.intensity;
    }
    bool operator > (const TargetInfo& right) const
    {
        return pos.intensity > right.pos.intensity;
    }
};

namespace Tracking
{
class GridTracking
{
public:
    GridTracking(ros::NodeHandle node, ros::NodeHandle private_node);
    ~GridTracking();
    void CallBack(const GridPointCloud::ConstPtr& msg);
    bool TargetFiltering(VPointCloud& cloud);
    void GenerateSpeed(std_msgs::Float64MultiArray& speed_msg, VPoint& pos, PointXYZ& sz);
    void PrintTargetList(vector<TargetInfo>& target_list, int pos);

private:
    ros::Subscriber map_subscriber;
    ros::Publisher cloud_publisher;
    ros::Publisher marker_publisher;
    //ros::Publisher path_publisher;
    ros::Publisher location_publisher;
    // bigdog Publisher
    ros::Publisher speed_publisher;
    ros::Publisher target_publisher;
    ros::Publisher cmd_vel_pub;
};
}

#endif
