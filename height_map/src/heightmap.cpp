// std includes
#include <iostream>
// #include <utility>
// ros includes
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
// #include <pcl/visualization/pcl_plotter.h>
// my includes
#include "height_map/heightmap.h"
#include "height_map/velodyne_calibration.hpp"
// dynamic configure
// #include <dynamic_reconfigure/server.h>
// #include <height_map/height_mapConfig.h>

using std::vector;
using std::cout;
using std::endl;
//using namespace pcl::visualization;

// pointclouds
VPointCloud obstacle1_cloud, clear1_cloud;
VPointCloud obstacle2_cloud, clear2_cloud;

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

namespace height_map
{
// double lidar_height;
// double lidar_angle;

// grid vectors
bool init[grid_dim][grid_dim];
double min_z[grid_dim][grid_dim];
double max_z[grid_dim][grid_dim];
double mean_z[grid_dim][grid_dim];
int num_pt[grid_dim][grid_dim];
int num_obs[grid_dim][grid_dim];
int num_clear[grid_dim][grid_dim];
vector<VPoint> obs_points[grid_dim][grid_dim];
vector<VPoint> clear_points[grid_dim][grid_dim];

// void ConfigCb(height_map::height_mapConfig &config, uint32_t level)
// {
//     lidar_angle = config.angle;
//     lidar_height = config.height;
// }

HeightMap::HeightMap(ros::NodeHandle node, ros::NodeHandle private_node)
{
	// get parameters using private node handle
	node.getParam("min_points", min_points);
	node.getParam("max_height", max_height);
	node.getParam("diff_terrain", diff_terrain);
	node.getParam("high_terrain", high_terrain);
	node.getParam("low_terrain", low_terrain);
	node.getParam("mean_threshold", mean_threshold);
	//node.getParam("lidar_height", lidar_height);
	//node.getParam("lidar_angle", lidar_angle);

    // dynamic_reconfigure
	// dynamic_reconfigure::Server<height_map::height_mapConfig> server;
	// dynamic_reconfigure::Server<height_map::height_mapConfig>::CallbackType f;
    //
	// f = boost::bind(&ConfigCb, _1, _2);
	// server.setCallback(f);

	// publish messages
	// calibration_publisher = node.advertise<VPointCloud>("lidar_calibration", 1);
	obstacle1_publisher = node.advertise<VPointCloud>("velodyne1_obstacles", 1);
	clear1_publisher = node.advertise<VPointCloud>("velodyne1_clear", 1);
    map1_publisher = node.advertise<GridPointCloud>("height1_map", 1);
	absolute1_publisher = node.advertise<GridPointCloud>("absolute1_map", 1);

	obstacle2_publisher = node.advertise<VPointCloud>("velodyne2_obstacles", 1);
	clear2_publisher = node.advertise<VPointCloud>("velodyne2_clear", 1);
    map2_publisher = node.advertise<GridPointCloud>("height2_map", 1);
	absolute2_publisher = node.advertise<GridPointCloud>("absolute2_map", 1);

	// subscribe to velodyne points
	velodyne1_scan = node.subscribe("velodyne1_calibration", 1, &HeightMap::ProcessData1,
									this, ros::TransportHints().tcpNoDelay(true));
	// subscribe to velodyne points
	velodyne2_scan = node.subscribe("velodyne2_calibration", 1, &HeightMap::ProcessData2,
	 								this, ros::TransportHints().tcpNoDelay(true));
    ros::spin();
}

HeightMap::~HeightMap() {}

/** point cloud input callback */
void HeightMap::ProcessData1(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // grid map header definition
	VPointCloud::Ptr scan(new VPointCloud);
	pcl::fromROSMsg(*msg,*scan);

	// point cloud calibration
    //VPointCloud::Ptr calibrated_cloud =
	//velodyne_calibration(scan, lidar_height, lidar_angle/180*M_PI);
    //calibration_publisher.publish(*calibrated_cloud);

    //1st velodyne_map generation
    GridPointCloud velodyne_map;
    velodyne_map.header.frame_id = msg->header.frame_id;
    velodyne_map.header.stamp = msg->header.stamp;
    //ConstructAbsoluteMaps(scan, velodyne_map);
	ConstructGridMaps(scan, velodyne_map);

	// 2nd absolute_map generatation
	GridPointCloud absolute_map;
    absolute_map.header.frame_id = msg->header.frame_id;
    absolute_map.header.stamp = msg->header.stamp;
    //ConstructAbsoluteMaps(scan, absolute_map);

	//3rd cloud header definition
	obstacle1_cloud.header.frame_id = scan->header.frame_id;
	obstacle1_cloud.header.stamp = scan->header.stamp;
	obstacle1_cloud.header.seq = scan->header.seq;

	clear1_cloud.header.frame_id = scan->header.frame_id;
	clear1_cloud.header.stamp = scan->header.stamp;
	clear1_cloud.header.seq = scan->header.seq;

	size_t npoints = scan->points.size();
 	obstacle1_cloud.points.resize(npoints);
	clear1_cloud.points.resize(npoints);

	size_t obs_count = 0;
	size_t empty_count = 0;

    // full point cloud generation
	//ConstructFullClouds(scan, npoints, obs_count, empty_count);
    // grid point cloud generation
    ConstructGridClouds(scan, obstacle1_cloud, clear1_cloud, npoints, obs_count, empty_count);

	obstacle1_cloud.points.resize(obs_count);
	clear1_cloud.points.resize(empty_count);

	obstacle1_publisher.publish(obstacle1_cloud);
	clear1_publisher.publish(clear1_cloud);

    map1_publisher.publish(velodyne_map);
	absolute1_publisher.publish(absolute_map);
}

/** point cloud input callback */
void HeightMap::ProcessData2(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // grid map header definition
	VPointCloud::Ptr scan(new VPointCloud);
	pcl::fromROSMsg(*msg,*scan);

	// point cloud calibration
    //VPointCloud::Ptr calibrated_cloud =
	//velodyne_calibration(scan, lidar_height, lidar_angle/180*M_PI);
    //calibration_publisher.publish(*calibrated_cloud);

    //1st velodyne_map generation
    GridPointCloud velodyne_map;
    velodyne_map.header.frame_id = msg->header.frame_id;
    velodyne_map.header.stamp = msg->header.stamp;
    //ConstructAbsoluteMaps(scan, velodyne_map);
	ConstructGridMaps(scan, velodyne_map);

	// 2nd absolute_map generatation
	GridPointCloud absolute_map;
    absolute_map.header.frame_id = msg->header.frame_id;
    absolute_map.header.stamp = msg->header.stamp;
    //ConstructAbsoluteMaps(scan, absolute_map);

	//3rd cloud header definition
	obstacle2_cloud.header.frame_id = scan->header.frame_id;
	obstacle2_cloud.header.stamp = scan->header.stamp;
	obstacle2_cloud.header.seq = scan->header.seq;

	clear2_cloud.header.frame_id = scan->header.frame_id;
	clear2_cloud.header.stamp = scan->header.stamp;
	clear2_cloud.header.seq = scan->header.seq;

	size_t npoints = scan->points.size();
 	obstacle2_cloud.points.resize(npoints);
	clear2_cloud.points.resize(npoints);

	size_t obs_count = 0;
	size_t empty_count = 0;

    // full point cloud generation
	//ConstructFullClouds(scan, npoints, obs_count, empty_count);
    // grid point cloud generation
    ConstructGridClouds(scan, obstacle2_cloud, clear2_cloud, npoints, obs_count, empty_count);

	obstacle2_cloud.points.resize(obs_count);
	clear2_cloud.points.resize(empty_count);

	obstacle2_publisher.publish(obstacle2_cloud);
	clear2_publisher.publish(clear2_cloud);

    map2_publisher.publish(velodyne_map);
	absolute2_publisher.publish(absolute_map);
}

void HeightMap::ConstructGridMaps(const VPointCloud::ConstPtr& scan,
	                              GridPointCloud& velodyne_map)
{
	clock_t start = clock();
	// initialization
	for (int x = 0; x < grid_dim; x++)
	{
		for (int y = 0; y < grid_dim; y++)
		{
			init[x][y] = false;
            min_z[x][y] = 0;
            max_z[x][y] = 0;
			mean_z[x][y] = 0;
            num_pt[x][y] = 0;
			num_obs[x][y] = 0;
			num_clear[x][y] = 0;
            obs_points[x][y].clear();
            clear_points[x][y].clear();
		}
	}

  	// max, min calculation
	for (unsigned i = 0; i < scan->points.size(); ++i)
	{
		int x = grid_dim / 2 + scan->points[i].x / cell_size;
		int y = grid_dim / 2 + scan->points[i].y / cell_size;
        double z = scan->points[i].z;
		if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim
			&& z < max_height)
		{
			if (!init[x][y])
			{
				min_z[x][y] = scan->points[i].z;
				max_z[x][y] = scan->points[i].z;
				init[x][y] = true;
			}
			else
			{
				min_z[x][y] = MIN(min_z[x][y], scan->points[i].z);      // min
				max_z[x][y] = MAX(max_z[x][y], scan->points[i].z);      // max
			}
		}
  	}

  	// calculate number of obstacles in each cell
	for (unsigned i = 0; i < scan->points.size(); ++i)
	{
		int x = grid_dim / 2 + scan->points[i].x / cell_size;
		int y = grid_dim / 2 + scan->points[i].y / cell_size;
        double z = scan->points[i].z;
		if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim
			&& z < max_height && init[x][y])
		{
            if((max_z[x][y] - min_z[x][y] > diff_terrain))
			{
				mean_z[x][y] += z;                  // mean
                num_pt[x][y]++;

				VPoint p;
				p.x = scan->points[i].x;
				p.y = scan->points[i].y;
				p.z = scan->points[i].z;
                p.ring = scan->points[i].ring;
                p.intensity = scan->points[i].intensity;
				obs_points[x][y].push_back(p);      // points
				num_obs[x][y]++;                    // num_obs
			}
			else
			{
				mean_z[x][y] += z;                  // mean
                num_pt[x][y]++;

                VPoint p;
                p.x = scan->points[i].x;
                p.y = scan->points[i].y;
                p.z = scan->points[i].z;
                p.ring = scan->points[i].ring;
                p.intensity = scan->points[i].intensity;
                clear_points[x][y].push_back(p);
				num_clear[x][y]++;                  // num_clear
			}
		}
	}

    // create clouds from grid
	for (int x = 0; x < grid_dim; x++)
	{
		for (int y = 0; y < grid_dim; y++)
		{
			if (num_obs[x][y] > min_points)
			{
				grid_cell cell;
				cell.x = x;
				cell.y = y;
				cell.num_obs = num_obs[x][y];
				cell.min_z = min_z[x][y];
				cell.max_z = max_z[x][y];
				cell.mean_z = mean_z[x][y] / num_pt[x][y];
				for(int  i = 0; i < obs_points[x][y].size(); i ++)
				{
					cell.p_x.push_back(obs_points[x][y][i].x);
					cell.p_y.push_back(obs_points[x][y][i].y);
					cell.p_z.push_back(obs_points[x][y][i].z);
                    cell.p_r.push_back(obs_points[x][y][i].ring);
                    cell.p_i.push_back(obs_points[x][y][i].intensity);
				}
				velodyne_map.cells.push_back(cell);
			}
            if(num_obs[x][y] > 0 && num_clear[x][y] > 0)
			{
                cout << "both obs and clear points: " << x << " " << y << endl;
			}
		}
	}
	clock_t end = clock();
	//cout << (end - start) * 1.0 / CLOCKS_PER_SEC * 1000 << " ms, Relative height_map time" << endl;
}

void HeightMap::ConstructAbsoluteMaps(const VPointCloud::ConstPtr& scan,
	                              GridPointCloud& velodyne_map)
{
	clock_t start = clock();
	// initialization
	for (int x = 0; x < grid_dim; x++)
	{
		for (int y = 0; y < grid_dim; y++)
		{
			init[x][y] = false;
            min_z[x][y] = 0;
            max_z[x][y] = 0;
			mean_z[x][y] = 0;
            num_pt[x][y] = 0;
			num_obs[x][y] = 0;
			num_clear[x][y] = 0;
            obs_points[x][y].clear();
            clear_points[x][y].clear();
		}
	}

  	// max, min calculation
	for (unsigned i = 0; i < scan->points.size(); ++i)
	{
		int x = grid_dim / 2 + scan->points[i].x / cell_size;
		int y = grid_dim / 2 + scan->points[i].y / cell_size;
        double z = scan->points[i].z;
		if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim
			&& z < max_height)
		{
			if (!init[x][y])
			{
				min_z[x][y] = scan->points[i].z;
				max_z[x][y] = scan->points[i].z;
				init[x][y] = true;
			}
			else
			{
				min_z[x][y] = MIN(min_z[x][y], scan->points[i].z);      // min
				max_z[x][y] = MAX(max_z[x][y], scan->points[i].z);      // max
			}
		}
  	}

  	// calculate number of obstacles in each cell
	for (unsigned i = 0; i < scan->points.size(); ++i)
	{
		int x = grid_dim / 2 + scan->points[i].x / cell_size;
		int y = grid_dim / 2 + scan->points[i].y / cell_size;
        double z = scan->points[i].z;
		if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim
			&& z < max_height && init[x][y])
		{
            if((max_z[x][y] - min_z[x][y] > diff_terrain)
			  ||max_z[x][y] > high_terrain
              ||min_z[x][y] < low_terrain)
			{
				mean_z[x][y] += z;                  // mean
                num_pt[x][y]++;

				VPoint p;
				p.x = scan->points[i].x;
				p.y = scan->points[i].y;
				p.z = scan->points[i].z;
                p.ring = scan->points[i].ring;
                p.intensity = scan->points[i].intensity;
				obs_points[x][y].push_back(p);      // points
				num_obs[x][y]++;                    // num_obs
			}
			else
			{
				mean_z[x][y] += z;                  // mean
                num_pt[x][y]++;

                VPoint p;
                p.x = scan->points[i].x;
                p.y = scan->points[i].y;
                p.z = scan->points[i].z;
                p.ring = scan->points[i].ring;
                p.intensity = scan->points[i].intensity;
                clear_points[x][y].push_back(p);
				num_clear[x][y]++;                  // num_clear
			}
		}
	}

    // create clouds from grid
	for (int x = 0; x < grid_dim; x++)
	{
		for (int y = 0; y < grid_dim; y++)
		{
			if (num_obs[x][y] > min_points)
			{
				grid_cell cell;
				cell.x = x;
				cell.y = y;
				cell.num_obs = num_obs[x][y];
				cell.min_z = min_z[x][y];
				cell.max_z = max_z[x][y];
				cell.mean_z = mean_z[x][y] / num_pt[x][y];
				for(int  i = 0; i < obs_points[x][y].size(); i ++)
				{
					cell.p_x.push_back(obs_points[x][y][i].x);
					cell.p_y.push_back(obs_points[x][y][i].y);
					cell.p_z.push_back(obs_points[x][y][i].z);
                    cell.p_r.push_back(obs_points[x][y][i].ring);
                    cell.p_i.push_back(obs_points[x][y][i].intensity);
				}
				velodyne_map.cells.push_back(cell);
			}
            if(num_obs[x][y] > 0 && num_clear[x][y] > 0)
			{
                cout << "both obs and clear points: " << x << " " << y << endl;
			}
		}
	}
	clock_t end = clock();
	cout << (end - start) * 1.0 / CLOCKS_PER_SEC * 1000 << " ms, Absolute height_map time" << endl;
}

void HeightMap::ConstructGridClouds(const VPointCloud::ConstPtr& scan,
	            VPointCloud& obstacle_cloud, VPointCloud& clear_cloud,
	            unsigned npoints, size_t& obs_count, size_t& empty_count)
{
	// initialization
	for (int x = 0; x < grid_dim; x++)
	{
		for (int y = 0; y < grid_dim; y++)
		{
			init[x][y] = false;
            min_z[x][y] = 0;
            max_z[x][y] = 0;
            mean_z[x][y] = 0;
            num_pt[x][y] = 0;
			num_obs[x][y] = 0;
			num_clear[x][y] = 0;
		}
	}

  	// max, min, mean and num_pt calculation
	for (unsigned i = 0; i < npoints; ++i)
	{
		int x = grid_dim / 2 + scan->points[i].x / cell_size;
		int y = grid_dim / 2 + scan->points[i].y / cell_size;
        double z = scan->points[i].z;
		if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim && z < max_height)
		{
			if (!init[x][y])
			{
				min_z[x][y] = scan->points[i].z;
				max_z[x][y] = scan->points[i].z;
                mean_z[x][y] += scan->points[i].z;
                num_pt[x][y] ++;
				init[x][y] = true;
			}
			else
			{
				min_z[x][y] = MIN(min_z[x][y], scan->points[i].z);
				max_z[x][y] = MAX(max_z[x][y], scan->points[i].z);
                mean_z[x][y] += scan->points[i].z;
                num_pt[x][y] ++;
			}
		}
  	}
    for (int x = 0; x < grid_dim; x++)
		for (int y = 0; y < grid_dim; y++)
            if (num_pt[x][y] > 0)
                mean_z[x][y] = mean_z[x][y] / num_pt[x][y];

  	// num_obs and num_clear calculation
	for (unsigned i = 0; i < npoints; ++i)
	{
		int x = grid_dim / 2 + scan->points[i].x / cell_size;
		int y = grid_dim / 2 + scan->points[i].y / cell_size;
        double z = scan->points[i].z;
		if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim && z < max_height
            && init[x][y])
		{
			if ((max_z[x][y] - min_z[x][y] > diff_terrain)
			  // || max_z[x][y] > high_terrain
			  // || min_z[x][y] < low_terrain
		      // ||mean_z[x][y] > mean_threshold
			)
			{

				num_obs[x][y]++;
                num_clear[x][y] = 0;
			}
			else
			{
				num_clear[x][y]++;
			}
		}
	}

  	// grid cloud calculation
	for (int x = 0; x < grid_dim; x++)
	{
		for (int y = 0; y < grid_dim; y++)
		{
			if (num_obs[x][y] > min_points)
			{
				obstacle_cloud.points[obs_count].x =
				-grid_dim / 2.0 * cell_size + x * cell_size + cell_size / 2.0;
				obstacle_cloud.points[obs_count].y =
				-grid_dim / 2.0 * cell_size + y * cell_size + cell_size / 2.0;
				obstacle_cloud.points[obs_count].z = diff_terrain;
				obs_count++;
			}
			if (num_clear[x][y] > 0)
			{
				clear_cloud.points[empty_count].x =
				-grid_dim / 2.0 * cell_size + x * cell_size + cell_size / 2.0;
				clear_cloud.points[empty_count].y =
				-grid_dim / 2.0 * cell_size + y * cell_size + cell_size / 2.0;
				clear_cloud.points[empty_count].z = diff_terrain;
				empty_count++;
			}
		}
	}
    obstacle_cloud.points.resize(obs_count);
    clear_cloud.points.resize(empty_count);
}
/*
void HeightMap::ConstructFullClouds(const VPointCloud::ConstPtr& scan,
	            unsigned npoints, size_t& obs_count, size_t& empty_count)
{
	// ROS_INFO_STREAM("fullClouds called.");
	memset(&init, 0, grid_dim * grid_dim);
    memset(&min_z, 0, grid_dim * grid_dim);
    memset(&max_z, 0, grid_dim * grid_dim);
    memset(&mean_z, 0, grid_dim * grid_dim);
    memset(&num_pt, 0, grid_dim * grid_dim);

	// calculate height map parameters
	for (unsigned i = 0; i < npoints; ++i)
	{
		int x = grid_dim / 2 + scan->points[i].x / cell_size;
		int y = grid_dim / 2 + scan->points[i].y / cell_size;
        double z = scan->points[i].z;
		if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim && z < max_height)
		{
			if (!init[x][y])
			{
				min_z[x][y] = scan->points[i].z;
				max_z[x][y] = scan->points[i].z;
                mean_z[x][y] += scan->points[i].z;
                num_pt[x][y]++;
				init[x][y] = true;
			}
			else
			{
				min_z[x][y] = MIN(min_z[x][y], scan->points[i].z);
				max_z[x][y] = MAX(max_z[x][y], scan->points[i].z);
                mean_z[x][y] += scan->points[i].z;
                num_pt[x][y]++;
			}
		}
	}
    for (int x = 0; x < grid_dim; x++)
        for (int y = 0; y < grid_dim; y++)
            mean_z[x][y] = mean_z[x][y] / num_pt[x][y];

  	// create full clouds from gridmap
	for (unsigned i = 0; i < npoints; ++i)
	{
		int x = grid_dim/2 + scan->points[i].x / cell_size;
		int y = grid_dim/2 + scan->points[i].y / cell_size;
        double z = scan->points[i].z;

		if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim && z < max_height
            && init[x][y])
		{
			if ((max_z[x][y] - min_z[x][y] > diff_terrain))
			//|| max_z[x][y] > high_terrain
			//|| min_z[x][y] < low_terrain)
			//||mean_z[x][y] > mean_threshold)
			{
				obstacle_cloud.points[obs_count].x = scan->points[i].x;
				obstacle_cloud.points[obs_count].y = scan->points[i].y;
				obstacle_cloud.points[obs_count].z = scan->points[i].z;
				obstacle_cloud.points[obs_count].ring = scan->points[i].ring;
				obstacle_cloud.points[obs_count].intensity = scan->points[i].intensity;
				obs_count++;
			}
			else
			{
				clear_cloud.points[empty_count].x = scan->points[i].x;
				clear_cloud.points[empty_count].y = scan->points[i].y;
				clear_cloud.points[empty_count].z = scan->points[i].z;
				clear_cloud.points[empty_count].ring = scan->points[i].ring;
				clear_cloud.points[empty_count].intensity = scan->points[i].intensity;
				empty_count++;
			}
		}
	}
    obstacle_cloud.points.resize(obs_count);
    clear_cloud.points.resize(empty_count);
    //std::cout << scan->points.size() << " " << obs_count + empty_count << " fullClouds_count " << std::endl;
}
*/
} // namespace height_map
