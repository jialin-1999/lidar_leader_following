#define PCL_NO_PRECOMPILE
// std include
#include <time.h>
#include <vector>
#include <utility>
// my include
#include "grid_tracking/GridFeature.hpp"
// pcl include
#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>

using namespace Eigen;
using namespace pcl;
using std::vector;
using std::pair;
using std::cout;
using std::endl;

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

namespace Tracking
{
    GridFeature::GridFeature(){}
    GridFeature::~GridFeature(){}

    void GridFeature::ClusterPosition(VPointCloud& cloud, PointXYZ& pt)
    {
        pt.x = 0; pt.y = 0; pt.z = 0;
        for(size_t i = 0; i < cloud.points.size(); i ++)
        {
            double x = cloud.points[i].x,
                   y = cloud.points[i].y,
                   z = cloud.points[i].z;
            pt.x += x; pt.y += y; pt.z += z;
        }
        pt.x /= cloud.points.size();
        pt.y /= cloud.points.size();
        pt.z /= cloud.points.size();
    }

    void GridFeature::ClusterSize(VPointCloud& cloud, PointXYZ& sz)
    {
        double max_x = -1000, min_x = 1000, max_y = -1000,
               min_y =1000, max_z = -1000, min_z = 1000;
        for(size_t i = 0; i < cloud.points.size(); i ++)
        {
            double x = cloud.points[i].x,
                   y = cloud.points[i].y,
                   z = cloud.points[i].z;
            if(x > max_x) max_x = x;
            if(x < min_x) min_x = x;
            if(y > max_y) max_y = y;
            if(y < min_y) min_y = y;
            if(z > max_z) max_z = z;
            if(z < min_z) min_z = z;
        }
        sz.x = max_x - min_x;
        sz.y = max_y - min_y;
        sz.z = max_z - min_z;
    }

    void GridFeature::AllMeanIntensity(VPointCloud& cloud, VPoint& pt)
    {
        pt.x = 0; pt.y = 0; pt.z = 0; pt.intensity = 0;
        size_t count = 0;
        for(size_t i = 0; i < cloud.points.size(); i ++)
        {
            double x = cloud.points[i].x,
                   y = cloud.points[i].y,
                   z = cloud.points[i].z,
                   intensity = cloud.points[i].intensity;
            pt.x += x; pt.y += y; pt.z += z;
            pt.intensity += intensity;
            count++;
        }
        if(count)
        {
            pt.x /= count;
            pt.y /= count;
            pt.z /= count;
            pt.intensity /= count;
        }
        else
        {
            pt.x = 0; pt.y = 0; pt.z = 0;
            pt.intensity = 0;
        }
    }

    void GridFeature::PartMeanIntensity(VPointCloud& cloud, VPoint& pt, 
                                        size_t& cnt)
    {
        pt.x = 0; pt.y = 0; pt.z = 0; pt.intensity = 0;
        size_t intensity_count = 0, pos_count = 0, cnt_100 = 0;
        for(size_t i = 0; i < cloud.points.size(); i ++)
        {
            double x = cloud.points[i].x,
                   y = cloud.points[i].y,
                   z = cloud.points[i].z,
                   intensity = cloud.points[i].intensity;
            pt.x += x; pt.y += y; pt.z += z;
            pos_count++;
            if(intensity >= 75 && z >= 0.3)
            {
                pt.intensity += intensity;
                intensity_count++;
                if(intensity >= 100)
                {
		          cnt_100 ++;
                }
            }
        }
        cnt = cnt_100;
        if(pos_count)
        {
            pt.x /= pos_count;
            pt.y /= pos_count;
            pt.z /= pos_count;
        }
        else
        {
            pt.x = 0; pt.y = 0; pt.z = 0;
        }
        if(intensity_count)
        {
            pt.intensity /= intensity_count;
        }
        else
        {
            pt.intensity = 0;
        }
    }

    void GridFeature::HundredIntensity(VPointCloud& cloud, size_t& count)
    {
        count = 0;
        for(size_t i = 0; i < cloud.points.size(); i ++)
        {
            double intensity = cloud.points[i].intensity;
            if(intensity == 100)
                count++;
        }
    }

    void GridFeature::IntensityHistogram(VPointCloud& cloud, vector<double>& histogram)
    {
        for(size_t i = 0; i < 26; i++)
            histogram.push_back(0);
        for(size_t i = 0; i < cloud.points.size(); i++)
        {
            histogram[cloud.points[i].intensity / 10] ++;
        }
    }
}
