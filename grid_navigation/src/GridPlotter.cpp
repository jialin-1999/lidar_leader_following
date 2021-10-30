#include "grid_navigation/GridPlotter.hpp"

namespace Grid_Navigation
{
GridPlotter::GridPlotter(){}

GridPlotter::~GridPlotter(){}

void GridPlotter::DirectMarkers(visualization_msgs::MarkerArray& marker, vector<double>& yaw_list)
{
    // if(yaw_list.size() == 1)
    //     return;
    marker.markers.clear();
    marker.markers.resize(yaw_list.size());
    for(size_t i = 0; i != marker.markers.size(); i++)
    {
        // if(i != marker.markers.size() - 1)
        // {
        //     continue;
        // }
        marker.markers[i].header.frame_id = "velodyne";
        marker.markers[i].header.stamp = ros::Time::now();

        marker.markers[i].ns = "direct_markers";
        marker.markers[i].id = i;
        marker.markers[i].type = visualization_msgs::Marker::ARROW;
        marker.markers[i].action = visualization_msgs::Marker::MODIFY;

        // Set the pose of the marker
        marker.markers[i].pose.position.x = 0;
        marker.markers[i].pose.position.y = 0;
        marker.markers[i].pose.position.z = 0;
        marker.markers[i].pose.orientation.x = 0;
        marker.markers[i].pose.orientation.y = 0;
        marker.markers[i].pose.orientation.z = sin((yaw_list[i] - M_PI) / 2);
        marker.markers[i].pose.orientation.w = cos((yaw_list[i] - M_PI) / 2);

        // Set the scale of the marker
        marker.markers[i].scale.x = 0.4;
        marker.markers[i].scale.y = 0.02;
        marker.markers[i].scale.z = 0.02;

        // Set the color and alpha
        marker.markers[i].color.r = 1.0f;
        marker.markers[i].color.g = 1.0f;
        marker.markers[i].color.b = 1.0f;
        marker.markers[i].color.a = 1.0;

        if(i == marker.markers.size() - 1)
        {
            marker.markers[i].scale.x = 0.4;
            marker.markers[i].scale.y = 0.03;
            marker.markers[i].scale.z = 0.03;

            marker.markers[i].color.r = 0.0f;
            marker.markers[i].color.g = 1.0f;
            marker.markers[i].color.b = 0.0f;
            marker.markers[i].color.a = 1.0;
        }
        if(marker.markers.size() < 2)
        {
            marker.markers[i].scale.x = 0.1;
            marker.markers[i].scale.y = 0.03;
            marker.markers[i].scale.z = 0.03;

            marker.markers[i].color.r = 0.0f;
            marker.markers[i].color.g = 0.0f;
            marker.markers[i].color.b = 0.0f;
            marker.markers[i].color.a = 1.0;
        }
        marker.markers[i].lifetime = ros::Duration();
    }
}

void GridPlotter::ObstacleMarkers(visualization_msgs::MarkerArray& marker, PointXYZ nearest_obstacle[])
{
    marker.markers.clear();
    marker.markers.resize(360);
    for(size_t i = 0; i != marker.markers.size(); i++)
    {
        marker.markers[i].header.frame_id = "velodyne";
        marker.markers[i].header.stamp = ros::Time::now();

        marker.markers[i].ns = "obstacle_markers";
        marker.markers[i].id = i;
        marker.markers[i].type = visualization_msgs::Marker::CUBE;
        marker.markers[i].action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker
        marker.markers[i].pose.position.x = nearest_obstacle[i].x;
        marker.markers[i].pose.position.y = nearest_obstacle[i].y;
        marker.markers[i].pose.position.z = 0;
        marker.markers[i].pose.orientation.x = 0;
        marker.markers[i].pose.orientation.y = 0;
        marker.markers[i].pose.orientation.z = 0;
        marker.markers[i].pose.orientation.w = 1;

        // Set the scale of the marker
        marker.markers[i].scale.x = 0.05;
        marker.markers[i].scale.y = 0.05;
        marker.markers[i].scale.z = 0.05;

        // Set the color and alpha
        marker.markers[i].color.r = 1.0f;
        marker.markers[i].color.g = 1.0f;
        marker.markers[i].color.b = 1.0f;
        marker.markers[i].color.a = 0.9;

        marker.markers[i].lifetime = ros::Duration();
    }
}
}
