//std includes
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
// 3rd party library
#include <Eigen/Core>
//ros includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
//pcl includes
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// my includes
#include "height_map/grid_cell.h"
#include "height_map/grid_map.h"
#include "grid_navigation/grid_size.hpp"

using std::vector;
using std::string;

// const parameters
const int sector_dim = 360;

typedef height_map::grid_map GridPointCloud;

namespace Grid_Navigation
{
class Navigation
{
public:
    Navigation(ros::NodeHandle node, ros::NodeHandle private_node);

    ~Navigation();

    // CallBack
    void positions_callback(const std_msgs::Float64MultiArray &msg);
    // CallBack
    void obstacles_callback(const GridPointCloud::ConstPtr& msg);

    // primary_histogram
    void BuildPrimaryHistogram();

    void GridPrimaryHistogram(int posx, int posy);

    void GridMagnitude(int posx, int posy);

    double GridTheta(int posx, int posy);

    // Binary Histogram
    void BuildBinaryHistogram();

    // masked_histogram
    void BuildMaskedHistogram();

    bool DriveRight(int posx, int poxy);

    bool DriveLeft(int posx, int posy);

    // Steering Direction
    void SteeringDirectionList();

    void SelectSteeringDirection();

    int DirectionCost(int sector);

private:
    ros::Subscriber map_subscriber;
    ros::Subscriber target_subscriber;
    ros::Publisher direct_publisher;
    ros::Publisher obstacle_publisher;
};
}
