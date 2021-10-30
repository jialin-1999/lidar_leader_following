#ifndef VELODYNE_CALIBRATION_
#define VELODYNE_CALIBRATION_

// A simple way to generate external calibartion parameters for velodyne VLP-16.
// 2016.11

// pcl includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
// my includes
#include "velodyne_type.hpp"

using namespace Eigen;

pcl::PointCloud<pcl::PointXYZ>::Ptr
velodyne_calibration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, double height = 0, double theta = 0)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);

	Matrix3f R;
	R << cos(theta), 0, -sin(theta),
		 0, 1, 0,
		sin(theta), 0, cos(theta);

	MatrixXf T(3,1);
	T << 0, 0, height;

	MatrixXf transform(4,4);
	transform << R, T,
	MatrixXf::Zero(1, 3),
	MatrixXf::Identity(1, 1);

	pcl::transformPointCloud(*cloudIn, *cloudOut, transform);
	return cloudOut;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr
velodyne_calibration(pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloudIn, double height = 0, double theta = 0)
{
	pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloudOut(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);

	Matrix3f R;
	R << cos(theta), 0, -sin(theta),
		 0, 1, 0,
		sin(theta), 0, cos(theta);

	MatrixXf T(3,1);
	T << 0, 0, height;

	MatrixXf transform(4,4);
	transform << R, T,
	MatrixXf::Zero(1, 3),
	MatrixXf::Identity(1, 1);

	pcl::transformPointCloud(*cloudIn, *cloudOut, transform);
	return cloudOut;
}

pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr
velodyne_calibration(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr& cloudIn, double height = 0, double theta = 0)
{
	pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloudOut(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);

	Matrix3f R;
	R << cos(theta), 0, -sin(theta),
		 0, 1, 0,
		sin(theta), 0, cos(theta);

	MatrixXf T(3,1);
	T << 0, 0, height;

	MatrixXf transform(4,4);
	transform << R, T,
	MatrixXf::Zero(1, 3),
	MatrixXf::Identity(1, 1);

	pcl::transformPointCloud(*cloudIn, *cloudOut, transform);
	return cloudOut;
}

#endif
