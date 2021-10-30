/*
height map v1
Construct a height map for grid navigation,
velodyne_obstacle and velodyne_clear.
2016.12
height map v2
Self-defined message grid_map.
2017.04
height_map v3
high obstruction detection with num_obs and
low obstruction detection with num_clear.
2017.05
*/

#include <ros/ros.h>
#include <height_map/heightmap.h>
/*
#include <dynamic_reconfigure/server.h>
#include <height_map/height_mapConfig.h>

double height;
double angle;

void ConfigCb(height_map::height_mapConfig &config, uint32_t level)
{
    angle = config.angle;
    height = config.height;
}
*/
/** Main entry point. */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "heightmap_node");
	ros::NodeHandle node;
	ros::NodeHandle private_nh("~");
/*
	// dynamic_reconfigure
	dynamic_reconfigure::Server<height_map::height_mapConfig> server;
	dynamic_reconfigure::Server<height_map::height_mapConfig>::CallbackType f;

	f = boost::bind(&ConfigCb, _1, _2);
	server.setCallback(f);
*/
	height_map::HeightMap hm(node, private_nh);

	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
	//ros::spin();
	return 0;
}
