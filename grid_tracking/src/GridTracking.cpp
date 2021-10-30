#define PCL_NO_PRECOMPILE
// std includes
#include <vector>
#include <algorithm>
#include <time.h>
#include <cmath>
// ros includes
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/visualization/histogram_visualizer.h>
// my includes
#include "grid_tracking/GridTracking.hpp"
#include "grid_tracking/GridCluster.hpp"
#include "grid_tracking/GridFeature.hpp"
#include "grid_tracking/GridPlotter.hpp"

using namespace std;

Tracking::GridCluster grid_cluster;
Tracking::GridFeature grid_feature;
Tracking::GridPlotter grid_plotter;

// map data
int num_obs[grid_dim][grid_dim];
double mean_z[grid_dim][grid_dim];
double max_z[grid_dim][grid_dim];
double min_z[grid_dim][grid_dim];
double diff_z[grid_dim][grid_dim];
vector<VPoint> cell_points[grid_dim][grid_dim];

// cluster data
int cluster_id[grid_dim][grid_dim];
vector< vector< pair<int, int> > > cluster_list;
vector<VPointCloud> cloud_list;
VPointCloud scence_cloud;
double last_x, last_y;
int last_init;

// target data
nav_msgs::Path path;
TargetInfo last_target;
size_t error_count = 0;

double Distance(VPoint p1, VPoint p2)
{
    //cout << p1.x << " "<< p2.x << " " <<  p1.y << " " << p2.y << endl;
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// for debug
void PrintAll(VPointCloud& cloud)
{
    cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
    for(size_t i = 0; i < cloud.points.size(); i++)
    {
        cout << cloud.points[i].intensity << " " << cloud.points[i].z;
        if (cloud.points[i].z <= 0.6)
            cout << "66666666666666666666666666" << endl;
        else
            cout << endl;
    }
    cout << "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB" << endl;
}

// for printing
void PointsCount(VPointCloud& cloud)
{
    size_t n1 = 0, n2 = 0, n3 = 0;
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
        double t = cloud.points[i].intensity;
        if(t >= 120)
        {
            n1++; n2++; n3++;
        }
        else if(t >= 100)
        {
            n1++; n2++;
        }
        else if(t >= 80)
            n1++;
    }
    cout << "INTENSITY Statistics: " << n1 << " " << n2 << " " << n3 << endl;
}

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

namespace Tracking
{
// vector<ros::Publisher> marker_publisher2;
    GridTracking::GridTracking(ros::NodeHandle node, ros::NodeHandle private_node)
    {
        // Subscriber and Publishers
        map_subscriber = node.subscribe("height1_map", 10, &GridTracking::CallBack,
                                        this, ros::TransportHints().tcpNoDelay(true));

        // marker_publisher2.resize(5);
        last_init = 0;
        cloud_publisher = node.advertise<VPointCloud>("tracking_scence", 10);
        marker_publisher = node.advertise<visualization_msgs::Marker>("target_marker", 10);
       //  marker_publisher2[0] = node.advertise<visualization_msgs::Marker>("target_marker0", 10);
       // marker_publisher2[1] = node.advertise<visualization_msgs::Marker>("target_marker1", 10);
       //  marker_publisher2[2] = node.advertise<visualization_msgs::Marker>("target_marker2", 10);
       // marker_publisher2[3] = node.advertise<visualization_msgs::Marker>("target_marker3", 10);
       // marker_publisher2[4] = node.advertise<visualization_msgs::Marker>("target_marker4", 10);
        location_publisher = node.advertise<std_msgs::Float64MultiArray>("target_position", 10); // to navigation
        // path_publisher = node.advertise<nav_msgs::Path>("target_path", 10);
        speed_publisher = node.advertise<std_msgs::Float64MultiArray>("speed_control", 10); // to bigdog
        target_publisher = node.advertise<std_msgs::Float64MultiArray>("target", 10); // ////////////////////////////////////////////////////
        cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel",10);
        ros::spin();
    }

    GridTracking::~GridTracking(){}

    void GridTracking::CallBack(const GridPointCloud::ConstPtr& cloud)
    {
        clock_t start, end;
        start = clock();

//********************Initialization Starts***********************
        scence_cloud.header.frame_id = cloud->header.frame_id;
        // scence_cloud.header.stamp = cloud->header.stamp;
        memset(mean_z, 0, sizeof(mean_z));
        memset(max_z, 0, sizeof(max_z));
        memset(min_z, 0, sizeof(min_z));
        memset(diff_z, 0, sizeof(diff_z));
        memset(num_obs, 0, sizeof(num_obs));
        memset(cluster_id, -1, sizeof(cluster_id));
        cluster_list.clear();

        for(int i = 0; i < grid_dim; i ++)
            for(int j = 0; j < grid_dim; j ++)
                cell_points[i][j].clear();

        // data extraction
        for (size_t i = 0; i < cloud->cells.size(); i++)
        {
            int x = cloud->cells[i].x;
            int y = cloud->cells[i].y;

            mean_z[x][y] = cloud->cells[i].mean_z;
            max_z[x][y] = cloud->cells[i].max_z;
            min_z[x][y] = cloud->cells[i].min_z;
            diff_z[x][y] = cloud->cells[i].max_z - cloud->cells[i].min_z;
            num_obs[x][y] = cloud->cells[i].num_obs;

            for(int j = 0; j < cloud->cells[i].p_x.size(); j ++)
            {
                VPoint p;
                p.x = cloud->cells[i].p_x[j];
                p.y = cloud->cells[i].p_y[j];
                p.z = cloud->cells[i].p_z[j];
                p.ring = cloud->cells[i].p_r[j];
                p.intensity = cloud->cells[i].p_i[j];
                cell_points[x][y].push_back(p);
            }
        }

        // Cluster Generation
        grid_cluster.GenerateGrids(num_obs, cluster_id, cluster_list);
        grid_cluster.GenerateClouds(cluster_list, cell_points, cloud_list);
/*
        cout << "All the cluster:+++++++++++++++++++++++++++" << endl;
        for(size_t i = 0; i < cloud_list.size(); i++)
        {
            PointXYZ pos;
            grid_feature.ClusterPosition(cloud_list[i], pos);
            VPoint pt;
            grid_feature.PartMeanIntensity(cloud_list[i], pt);
            cout << i + 1 << " " << pos.x << " " << pos.y << " " << cloud_list[i].points.size()
            << " " << pt.intensity << endl;
        }
        cout << "All the cluster end.++++++++++++++++++++++++" << endl;
*/

        // scence cloud

        scence_cloud.clear();
        scence_cloud.height = 1;
        scence_cloud.width = 30000;
        scence_cloud.resize(scence_cloud.height * scence_cloud.width);
        //grid_plotter.IntensityClusters(target_list, scence_cloud);
        size_t count = 0;
        for(size_t i = 0; i < cloud_list.size(); i++)
        {
            for(size_t j = 0; j < cloud_list[i].points.size(); j++)
            {
                scence_cloud.points[count].x = cloud_list[i].points[j].x;
                scence_cloud.points[count].y = cloud_list[i].points[j].y;
                scence_cloud.points[count].z = cloud_list[i].points[j].z;
                count++;
            }
        }
        scence_cloud.points.resize(count);
        scence_cloud.width = count;

/*
        cout << "scence_cloud size: " << scence_cloud.points.size() << endl;
        cout << " scence_cloud: " << endl;
        for(size_t i = 0; i < scence_cloud.points.size(); i++)
        {
            cout << scence_cloud.points[i].x << " "
                 << scence_cloud.points[i].y << " "
                 << scence_cloud.points[i].z << " " << endl;
        }
*/

        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(scence_cloud, *cloud_msg);
        cloud_msg->header.frame_id = "velodyne";
        cloud_msg->header.stamp = ros::Time::now();
        cloud_publisher.publish(cloud_msg);

        if(cloud_list.size() == 0)
        {
            cout << "Empty Environment!" << endl;
            return;
        }
//*********************Initialization Ends*************************

        //1st Potential target list
        vector<TargetInfo> target_list;
        target_list.clear();
        for(size_t i = 0; i != cloud_list.size(); i++)
        {
            TargetInfo target_info;

            PointXYZ sz;
            grid_feature.ClusterSize(cloud_list[i], sz);
            VPoint pos;
            size_t int_cnt;
            grid_feature.PartMeanIntensity(cloud_list[i], pos, int_cnt);
            size_t count;
            //grid_feature.HundredIntensity(cloud_list[i], count);
            vector<double> histogram;
            grid_feature.IntensityHistogram(cloud_list[i], histogram);
/*
            cout << "??????????????????All the list start:?????????????????????????" << endl;
            for(size_t k = 0; k != cloud_list.size(); k++)
            {
                cout << k + 1 << ": " << cloud_list[k].pos.x << " "
                     << cloud_list[k].pos.y << " " << cloud_list[k].pos.intensity << endl;
            }
            cout << "???????????????????All the list end.????????????????????" << endl;
*/
            // SIZE elimination
            if((sz.x + sz.y < 0.2) || sz.z < 0.5 || sz.z > 2)
            {
                //cout << pos.x << " " << pos.y << " " << pos.intensity << endl;
                continue;
            }
            // INTENSITY elimination
            if(pos.intensity <= 75)
            {
                //cout << pos.x << " " << pos.y << " " << pos.intensity << endl;
                continue;
            }

            double dist = 0, mul = 0;
            dist = sqrt(pow(pos.x, 2) + pow(pos.y, 2));
            mul = dist * int_cnt;
            if(mul < 50)
            {
            printf("(x, y) = (%.3f %.3f) || %.3f\n", target_info.pos.x,
                                                     target_info.pos.y);
	        continue;
            }

	    // RATIO elimination
	    double ratio = int_cnt * 1.0 / cloud_list[i].points.size();
	    if(ratio < 0.3)
	    {
		continue;
	    }

            target_info.target = true;
            target_info.pos = pos;
            target_info.sz = sz;
            target_info.count = int_cnt;
            target_info.histogram = histogram;
            target_info.cloud = cloud_list[i];
            target_list.push_back(target_info);
        }

        sort(target_list.begin(), target_list.end(), greater<TargetInfo>());
/*
        cout << "--------------sort start:----------------" << endl;
        for(size_t i = 0; i != target_list.size(); i++)
        {
            cout << target_list[i].pos.intensity << " "
                 << target_list[i].pos.x << " " << target_list[i].pos.y << endl;
        }
        cout << endl;
        cout << "--------------sort end. -----------------" << endl;
*/
        //2nd target selection from list
        VPoint max_pos;
        PointXYZ max_sz;
        max_pos.x = 0; max_pos.y = 0; max_pos.z = 0;
        max_sz.x = 0; max_sz.y = 0; max_sz.z = 0;

        int pos_final = 0;
        geometry_msgs::Twist cmd_vel;
        if (target_list.size() == 0)
        {
            cout << "No target!" << endl;

            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;

            cmd_vel_pub.publish(cmd_vel);
            visualization_msgs::Marker no_marker;
            grid_plotter.PrintMarkers(no_marker, max_pos, max_sz);
            marker_publisher.publish(no_marker);




            std_msgs::Float64MultiArray location1;
            location1.data.push_back(0);
            location1.data.push_back(0);
            location1.data.push_back(0);
            target_publisher.publish(location1);






            return;
        }
        else if(target_list.size() == 1)
        {
            max_pos = target_list[0].pos;
            max_sz = target_list[0].sz;
            last_target.pos = max_pos;
            last_target.sz = max_sz;

	    VPoint pp;
            size_t cntt;
            grid_feature.PartMeanIntensity(target_list[0].cloud, pp, cntt);
	    cout << "******************ONE intensity count: ********************************" << endl;
            cout << pp.intensity << " " << target_list[0].count << " " << target_list[0].cloud.points.size() << " "
                 << target_list[0].count * 1.0 / target_list[0].cloud.points.size() << " "
                 << target_list[0].pos.x << " " <<  target_list[0].pos.y << endl;
            //PrintTargetList(target_list, 0);
            //PrintAll(target_list[0].cloud);






            std_msgs::Float64MultiArray location1;
            location1.data.push_back(target_list[0].pos.x);
            location1.data.push_back(target_list[0].pos.y);
            location1.data.push_back(sqrt(pow(target_list[0].pos.x, 2) + pow(target_list[0].pos.y, 2)) );
            target_publisher.publish(location1);


        }
        else
        {
	    cout << "******************Largest intensity count: ********************************" << endl;
            int pos1 = -1, pt1 = 0, max1 = 0;
            double min_dis = 100.0;
	    for(int i = 0; i < min(int(target_list.size()), 3); i++)
            {
	    	VPoint pp;
            	size_t cntt;
            	grid_feature.PartMeanIntensity(target_list[i].cloud, pp, cntt);

		double dist = 0, ratio = 0, mul = 0;
                dist = sqrt(pow(target_list[i].pos.x, 2) + pow(target_list[i].pos.y, 2));
                mul = dist * target_list[i].count;
                ratio = target_list[i].count * 1.0 / target_list[i].cloud.points.size();

            //         if(pp.intensity > max1)
		    // {
			// max1 = pp.intensity; pt1 = target_list[i].count, pos1 = i;
		    // }
              if (!last_init) {
                  if (pp.intensity > max1) {
                      max1 = pp.intensity;
                      pt1 = target_list[i].count;
                      pos1 = i;
                  }
              } else {
                  double tmp_dis =
                    sqrt((target_list[i].pos.x - last_x) * (target_list[i].pos.x - last_x) +
                         (target_list[i].pos.y - last_y) * (target_list[i].pos.y - last_y));
                  if (tmp_dis < min_dis) {
                    min_dis = tmp_dis;
                    pos1 = i;
                  }
              }

	        //if(target_list[i].count > pt1)
		    //pt1 = target_list[i].count, pos1 = i;

                cout << pp.intensity << " " << target_list[i].count << " " << target_list[i].cloud.points.size() << " "
		     << target_list[i].count * 1.0 / target_list[i].cloud.points.size() << " "
                     << target_list[i].pos.x << " " <<  target_list[i].pos.y << endl;
            }
            cout << "target pos: " << pos1 << endl;
	    ROS_ASSERT(pos1 != -1);
        last_init = 1;
        last_x = target_list[pos1].pos.x;
        last_y = target_list[pos1].pos.y;

	    // for(int i = 0; i < min(int(target_list.size()), 2); i++)
        //     {
	    //     if(i != pos1)
		// {
        //             visualization_msgs::Marker marker;
        //             grid_plotter.PrintMarkers(marker, target_list[i].pos, target_list[i].sz);
        //             marker.color.r = 0.0;
        //             marker.color.g = 0.0;
        //             marker.color.b = 0.0;
        //             marker_publisher2[i].publish(marker);
        //         }
        //     }

	    cout << endl;
	    max_pos = target_list[pos1].pos;
	    max_sz = target_list[pos1].sz;
	    pos_final = pos1;

            std_msgs::Float64MultiArray location1;
            location1.data.push_back(target_list[0].pos.x);
            location1.data.push_back(target_list[0].pos.y);
            location1.data.push_back(sqrt(pow(target_list[0].pos.x, 2) + pow(target_list[0].pos.y, 2)) );
            target_publisher.publish(location1);



	    // PrintTargetList(target_list, pos1);
            /*
            if(target_list[0].pos.intensity / target_list[1].pos.intensity > 1.25)
            {
                max_pos = target_list[0].pos;
                max_sz = target_list[0].sz;
                last_target.pos = max_pos;
                last_target.sz = max_sz;
                PrintTargetList(target_list, 0);
                //PrintAll(target_list[0].cloud);
            }
            else
            {
                // Distance
                double min_dis = 10000;
                int target_pos = 0;
                for(size_t i = 0; i < target_list.size(); i++)
                {
                    double dist = Distance(last_target.pos, target_list[i].pos);
                    if(min_dis > dist)
                    {
                        min_dis = dist;
                        target_pos = i;
                    }
                }

                max_pos = target_list[target_pos].pos;
                max_sz = target_list[target_pos].sz;
                pos_final = target_pos;
                PrintTargetList(target_list, target_pos);
                //PrintAll(target_list[target_pos].cloud);

                max_pos = target_list[0].pos;
                max_sz = target_list[0].sz;
                PrintTargetList(target_list, 0);

            }
            */
        }

        if(!TargetFiltering(target_list[pos_final].cloud))
        {
            max_pos.x = 0; max_pos.y = 0; max_pos.z = 0;
            max_sz.x = 0.02; max_sz.y = 0.02; max_sz.z = 0.02;
            //cout << "00000000000000000000000000000000000000000" << endl;
        }

        // target marker
        visualization_msgs::Marker marker;
        grid_plotter.PrintMarkers(marker, max_pos, max_sz);
        marker_publisher.publish(marker);

        // target location
        std_msgs::Float64MultiArray location;
        location.data.push_back(max_pos.x);
        location.data.push_back(max_pos.y);
        location_publisher.publish(location);
	    cout << location.data[0]<<" "<<location.data[1]<<endl;
/*
        // target path
        grid_plotter.PrintPaths(path, max_pos);
        path_publisher.publish(path);
*/

        // bigdog speed
        std_msgs::Float64MultiArray speed_msg;

        if(fabs(max_pos.x) + fabs(max_pos.y) < 0.1)
        {
            error_count++;
            if(error_count >= 50)
            {
                speed_msg.data.push_back(0);
                speed_msg.data.push_back(0);
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                GenerateSpeed(speed_msg, last_target.pos, last_target.sz);
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0.1;
                cmd_vel.angular.z = atan2(last_target.pos.y, last_target.pos.x);
            }
        }
        else
        {
            error_count = 0;
            GenerateSpeed(speed_msg, max_pos, max_sz);
            cmd_vel.linear.x = 0.1;
            cmd_vel.angular.z = atan2(max_pos.x,max_pos.y);

        }
        speed_publisher.publish(speed_msg);
        cmd_vel_pub.publish(cmd_vel);
        // cmd_vel.linear.x = 0.5;
        // cmd_vel.angular.z = atan2(last_target.pos.y, last_target.pos.x);
/*
        cout << "$$$$$$$$$$$$$$$$$$$$$$ publish speed: $$$$$$$$$$$$$$$$$$$$$$" << endl;
        cout << max_pos.x << " " << max_pos.y << " " << max_pos.z << endl;
        cout << max_sz.x << " " << max_sz.y << " " << max_sz.z << endl;
        cout << speed_msg.data[0] << " " << speed_msg.data[1] << endl;
*/
        end = clock();
        //cout << (end - start) * 1.0 / CLOCKS_PER_SEC * 1000 << " ms, GridTracking time" << endl;
    }

    bool GridTracking::TargetFiltering(VPointCloud& cloud)
    {
        // distance too far
        PointXYZ pt;
        grid_feature.ClusterPosition(cloud, pt);
        double dist = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        if(dist >= 6.0)
            return false;

        // intensity too low
        VPoint p;
        size_t cnt;
        grid_feature.PartMeanIntensity(cloud, p, cnt);
        if(p.intensity <= 100)
            return false;

        // high intensity points too small
        size_t n = 0;
        for (size_t i = 0; i < cloud.points.size(); i++)
        {
            double t = cloud.points[i].intensity;
            if(t >= 100)
            {
                n++;
            }
        }
        if (dist <= 3.5 && n <= 10)
            return false;
        else if(dist > 3.5 && dist <= 4.5 && n <= 6)
            return false;
        else if(dist >4.5 && n <=3)
            return false;
        else
            return true;
    }

    void GridTracking::GenerateSpeed(std_msgs::Float64MultiArray& speed_msg, VPoint& pos, PointXYZ& sz)
    {
        double dist = sqrt(pow(pos.x, 2) + pow(pos.y, 2));
        if(dist < 2.0)
        {
            speed_msg.data.push_back(0);
            speed_msg.data.push_back(0);
        }
        else
        {
            speed_msg.data.push_back(0.5);
            speed_msg.data.push_back(atan2(pos.y, pos.x));
        }
    }

    void GridTracking::PrintTargetList(vector<TargetInfo>& target_list, int pos)
    {
        //return;
        cout << "**************Target_info start*****************" << endl;
        cout << "Target mean intensity: " << target_list[pos].pos.intensity << endl;
        PointsCount(target_list[pos].cloud);

        cout << "Target position: ";
        cout << target_list[pos].pos.x << " "
             << target_list[pos].pos.y << " "
             << sqrt(pow(target_list[pos].pos.x, 2) + pow(target_list[pos].pos.y, 2)) << endl;

        cout << "Target angle: ";
        cout << atan2(target_list[pos].pos.y, target_list[pos].pos.x) / M_PI * 180 << endl;

        cout << "Target size: ";
        cout << target_list[pos].sz.x << " "
             << target_list[pos].sz.y << " "
             << target_list[pos].sz.z << " "
             << target_list[pos].sz.x * target_list[pos].sz.y * target_list[pos].sz.z << endl;

        cout << "Target histogram:" << endl;
        for(size_t i = 0; i < target_list[pos].histogram.size(); i++)
            cout << target_list[pos].histogram[i] << " ";
        cout << endl;
        cout << "--------------Target_info end--------------------" << endl;
    }
}
