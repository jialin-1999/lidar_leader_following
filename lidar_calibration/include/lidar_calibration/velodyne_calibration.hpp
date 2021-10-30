#ifndef VELODYNE_CALIBRATION_
#define VELODYNE_CALIBRATION_

// A simple way to calibarte velodyne VLP-16.
// 2016.11
// Add roll, yaw, and lidar_range for calibartion.
// 2017.07

// pcl includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
// my includes
#include "velodyne_type.hpp"

using namespace Eigen;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

typedef velodyne_pointcloud::PointXYZIR PointXYZIR;
typedef pcl::PointCloud<PointXYZIR> PointCloudXYZIR;

float DegToRad(float& angle)
{
    return angle / 180.0 * M_PI;
}

bool point_range(double x, double y, double z, double range)
{
    double d = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    if(d <= range && x >= 0 && y >= -3 && y <= 3 && z <= 2.5)
        return true;
    else
        return false;
}

void velodyne_calibration(PointCloudXYZ::Ptr cloudIn, PointCloudXYZ::Ptr cloudOut,
	 float height = 0, float pitch = 0, float roll = 0, float yaw = 0)
{
	pitch =DegToRad(pitch);
	roll = DegToRad(roll);
	yaw = DegToRad(yaw);

	Matrix3f R, MP, MR, MY;
	MP << cos(pitch), 0, -sin(pitch),
		  0, 1, 0,
		  sin(pitch), 0, cos(pitch);

	MR << 1, 0, 0,
	      0, cos(roll), -sin(roll),
		  0, sin(roll), cos(roll);

	MY << cos(yaw), -sin(yaw), 0,
	      sin(yaw), cos(yaw), 0,
		  0, 0, 1;

    R = MY * MR * MP;
	MatrixXf T(3,1);
	T << 0, 0, height;

	MatrixXf transform(4,4);
	transform << R, T,
	MatrixXf::Zero(1, 3),
	MatrixXf::Identity(1, 1);

	pcl::transformPointCloud(*cloudIn, *cloudOut, transform);
}

void velodyne_calibration(PointCloudXYZIR::Ptr cloudIn, PointCloudXYZIR::Ptr cloudOut,
	 float height = 0, float pitch = 0, float roll = 0, float yaw = 0)
{
    clock_t start, end;
	pitch = DegToRad(pitch);
	roll = DegToRad(roll);
	yaw = DegToRad(yaw);

	Matrix3f R, MP, MR, MY;
	MP << cos(pitch), 0, -sin(pitch),
		  0, 1, 0,
		  sin(pitch), 0, cos(pitch);

	MR << 1, 0, 0,
	      0, cos(roll), -sin(roll),
		  0, sin(roll), cos(roll);

	MY << cos(yaw), -sin(yaw), 0,
	      sin(yaw), cos(yaw), 0,
		  0, 0, 1;

    R = MY * MR * MP;
	MatrixXf T(3,1);
	T << 0, 0, height;

	MatrixXf transform(4,4);
	transform << R, T,
	MatrixXf::Zero(1, 3),
	MatrixXf::Identity(1, 1);

    start = clock();
	pcl::transformPointCloud(*cloudIn, *cloudOut, transform);
    end = clock();
    //std::cout << (end - start) * 1.0 / CLOCKS_PER_SEC * 1000 << " ms, Transform PointCloud time" << std::endl;
}

void velodyne_range(PointCloudXYZ::Ptr cloudIn, PointCloudXYZ::Ptr cloudOut,
                    double range = 0)
{
    cloudOut->points.resize(cloudIn->points.size());
    size_t range_sz = 0;
    if(fabs(range - 100) < 1)
        return;
    else
    {
        for(size_t i = 0; i < cloudIn->points.size(); i++)
        {
            double x = cloudIn->points[i].x;
            double y = cloudIn->points[i].y;
            double z = cloudIn->points[i].z;
            if(point_range(x, y, z, range))
            {
                cloudOut->points[range_sz].x = x;
                cloudOut->points[range_sz].y = y;
                cloudOut->points[range_sz].z = z;
                range_sz++;
            }
        }
        cloudOut->points.resize(range_sz);
    }
}

void velodyne_range(PointCloudXYZIR::Ptr cloudIn, PointCloudXYZIR::Ptr cloudOut,
                    double range = 0)
{
    cloudOut->points.resize(cloudIn->points.size());
    size_t range_sz = 0;
    if(fabs(range - 100) < 1)
        return;
    else
    {
        for(size_t i = 0; i < cloudIn->points.size(); i++)
        {
            double x = cloudIn->points[i].x;
            double y = cloudIn->points[i].y;
            double z = cloudIn->points[i].z;
            double r = cloudIn->points[i].ring;
            double n = cloudIn->points[i].intensity;
            if(point_range(x, y, z, range))
            {
                cloudOut->points[range_sz].x = x;
                cloudOut->points[range_sz].y = y;
                cloudOut->points[range_sz].z = z;
                cloudOut->points[range_sz].ring = r;
                cloudOut->points[range_sz].intensity = n;
                range_sz++;
            }
        }
        cloudOut->points.resize(range_sz);
    }
}

#endif
