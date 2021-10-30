#include "grid_tracking/GridPlotter.hpp"

#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

namespace Tracking
{
GridPlotter::GridPlotter(){}

GridPlotter::~GridPlotter(){}

void GridPlotter::PrintMarkers(visualization_msgs::Marker& marker, VPoint& pos, PointXYZ& sz)
{
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();

    marker.ns = "tracking";     // namespace
    marker.id = 0;              // id
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker
    marker.pose.position.x = pos.x;
    marker.pose.position.y = pos.y;
    marker.pose.position.z = pos.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = sz.x;
    marker.scale.y = sz.y;
    marker.scale.z = sz.z;

    // Set the color and alpha
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.9;

    marker.lifetime = ros::Duration();
}

void GridPlotter::PrintPaths(nav_msgs::Path& path, VPoint& pos)
{
    //cout << "PrintPaths called! " << pos.x << " " << pos.y << endl;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "velodyne";

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = pos.x;
    this_pose_stamped.pose.position.y = pos.y;
    double th = 0.0;
    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "velodyne";
    path.poses.push_back(this_pose_stamped);

    return;
}

void GridPlotter::IntensityClusters(vector<TargetInfo>& target_list, VPointCloud& scence_cloud)
{
    //std::cout << "Visulization called." << std::endl;
    scence_cloud.height = 1;
    scence_cloud.points.resize(30000);
    size_t count = 0;
    double x = 0, y = 0, z = 0;
    for(size_t i = 0; i != target_list.size(); i++)
    {
        //cout << "current list size:" << target_list[i].cloud->points.size() << endl;
        for(size_t j = 0; j != target_list[i].cloud.points.size(); j++)
        {
            x = target_list[i].cloud.points[j].x;
            y = target_list[i].cloud.points[j].y;
            z = target_list[i].cloud.points[j].z;

            scence_cloud.points[count].x = x;
            scence_cloud.points[count].y = y;
            scence_cloud.points[count].z = z;

            count++;
        }
    }

    scence_cloud.width = count;
    scence_cloud.points.resize(count);
    //cout << "scence_cloud size " << scence_cloud.points.size() << endl;
}
}
