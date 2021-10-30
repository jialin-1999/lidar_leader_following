// std includes
#include <iostream>
// ros includes
#include <ros/ros.h>
// my includes
#include "grid_navigation/GridNavigation.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_navigation_node");
	ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    Grid_Navigation::Navigation navi(node, private_node);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
	//ros::spin();
    return 0;
}
