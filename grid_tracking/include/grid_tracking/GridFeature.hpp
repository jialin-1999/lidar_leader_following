#ifndef GRID_FEATURE_H_
#define GRID_FEATURE_H_
// std includes
#include <iostream>
#include <vector>
// pcl includes
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Dense>
// my includes
#include "grid_tracking/velodyne_type.hpp"
#include "grid_tracking/grid_size.hpp"

using std::vector;
using std::pair;
using namespace Eigen;
using namespace pcl;

namespace Tracking
{
class GridFeature
{
public:
    GridFeature();
    ~GridFeature();

    void ClusterPosition(VPointCloud& cloud, PointXYZ& pt);
    void ClusterSize(VPointCloud& cloud, PointXYZ& sz);
    void HundredIntensity(VPointCloud& cloud, size_t& count);
    void AllMeanIntensity(VPointCloud& cloud, VPoint& pt);
    void PartMeanIntensity(VPointCloud& cloud, VPoint& pt, size_t& cnt);
    void IntensityHistogram(VPointCloud& cloud, vector<double>& histogram);

    //descriptor related
    void VFHSignature(VPointCloud& cloud, PointCloud<VFHSignature308>::Ptr vfhs);
};
}

#endif
