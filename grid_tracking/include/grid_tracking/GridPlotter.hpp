#include <iostream>
#include <vector>
#include <utility>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "grid_tracking/grid_size.hpp"
#include "grid_tracking/velodyne_type.hpp"
#include "grid_tracking/GridTracking.hpp"

using std::vector;
using std::pair;
using std::cout;
using std::endl;

namespace Tracking
{
class GridPlotter
{
public:
    GridPlotter();
    ~GridPlotter();

    void PrintMarkers(visualization_msgs::Marker& marker, VPoint& pos, PointXYZ& sz);
    void PrintPaths(nav_msgs::Path& path, VPoint& pos);
    void IntensityClusters(vector<TargetInfo>& target_list, VPointCloud& scence_cloud);
};

}
