#include <iostream>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "grid_navigation/grid_size.hpp"
#include "grid_navigation/velodyne_type.hpp"

using std::vector;
using std::pair;
using std::cout;
using std::endl;

namespace Grid_Navigation
{
class GridPlotter
{
public:
    GridPlotter();
    ~GridPlotter();

    void DirectMarkers(visualization_msgs::MarkerArray& marker, vector<double>& yaw);
    void ObstacleMarkers(visualization_msgs::MarkerArray& marker, PointXYZ nearest_obstacle[]);
};

}
