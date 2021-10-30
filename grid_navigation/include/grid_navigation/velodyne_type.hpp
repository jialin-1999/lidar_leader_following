/*
velodyne pointcloud type for processing.
 */

#ifndef VELODYNE_POINTCLOUD_POINT_TYPES_H
#define VELODYNE_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace velodyne_pointcloud
{
    /** Euclidean Velodyne coordinate, including intensity and ring number. */
    struct PointXYZIR
    {
        PCL_ADD_POINT4D;                    // quad-word XYZ
        float    intensity;                 // laser intensity reading
        uint16_t ring;                      // laser ring number
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
    } EIGEN_ALIGN16;

};

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointXYZRGB PointXYZRGB;
typedef pcl::PointCloud<PointXYZRGB> PointCloudRGB;
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

#endif // VELODYNE_POINTCLOUD_POINT_TYPES_H
