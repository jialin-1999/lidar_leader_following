// std includes
#include <iostream>
// ros includes
#include <ros/ros.h>
// my includes
#include "grid_tracking/GridTracking.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_tracking");
	ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    Tracking::GridTracking nd(node, private_node);

    return 0;
}
