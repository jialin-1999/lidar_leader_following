#include <iostream>
#include <string.h>
#include <utility>
#include "grid_tracking/GridCluster.hpp"

using std::cout;
using std::endl;

const int MIN_GRIDS = 3;
const int MIN_POINTS = 30;

namespace Tracking
{
    GridCluster::GridCluster(){}
    GridCluster::~GridCluster(){}

    void GridCluster::GenerateGrids(int num_obs[][grid_dim],      // input
         int cluster_id[][grid_dim],                              // output
         vector< vector< pair<int,int> > > &cluster_list)         // output
    {
        cluster_list.clear();
        memset(visit, 0, sizeof(visit));
        for(size_t i = 0; i < grid_dim; i++)
            for(size_t j = 0; j < grid_dim; j++)
                cluster_id[i][j] = -1;

        radius = grid_dim / 2;
    	if(grid_dim / 2 < radius)
    		radius = grid_dim / 2;
	    left = grid_dim / 2 - radius,
        right = grid_dim / 2 + radius;

        int cluster_count = 0;
        for(int i = left; i < right; i ++)
        {
            for(int j = left; j < right; j ++)
            {
                if(!visit[i][j])
                {
                    if(num_obs[i][j] > 0)
                    {
                        cluster_current.clear();
                        DepthFirstSearch(num_obs, i, j, cluster_count, cluster_id);
                        cluster_list.push_back(cluster_current);
                        cluster_count ++;
                    }
                    else
                         visit[i][j] = 1;
                }
            }
        }
        //cout << "cluster list size: " << cluster_list.size() << endl;
    }

    void GridCluster::DepthFirstSearch(int num_obs[][grid_dim], int &x, int &y,
         int &cluster_count,int cluster_id[][grid_dim])
    {
     	visit[x][y] = 1;
        cluster_id[x][y] = cluster_count;
    	cluster_current.push_back(std::make_pair(x,y));
        for(int i = 0; i < dir_num; i ++)
    	{
    	    int nextX = x + dir_pair[i][0];
    	    int nextY = y + dir_pair[i][1];
    	    if(nextX >= left && nextX < right && nextY >= left && nextY < right
                &&!visit[nextX][nextY] && num_obs[nextX][nextY] > 0)
    	        DepthFirstSearch(num_obs, nextX, nextY, cluster_count, cluster_id);
    	}
    }

    void GridCluster::GenerateClouds(vector< vector<pair<int,int> > >& cluster_list,
         vector<VPoint> cell_points[][grid_dim], vector<VPointCloud>& cloud_list)
    {
        cloud_list.clear();
        for (vector< vector< pair<int, int> > > :: iterator it = cluster_list.begin();
            it != cluster_list.end(); it ++)
        {
            // few grids
            if (it->size() < MIN_GRIDS)
                continue;

            //clock_t start_init = clock();
            VPointCloud::Ptr cloud(new VPointCloud);
            cloud->height = 1;
            cloud->width = 30000;
            cloud->is_dense = false;
            // reduce initial time
            size_t pt_sz = 0;
            for(vector< pair<int,int> > :: iterator iter = (*it).begin(); iter != (*it).end(); iter ++)
            {
                int x = iter->first, y = iter->second;
                pt_sz += cell_points[x][y].size();
            }
            // //clock_t end_init = clock();

            // few points
            if (pt_sz < MIN_POINTS)
                continue;
            else
                cloud->points.resize(pt_sz + 1);

            size_t count = 0;
            for(vector< pair<int,int> > :: iterator iter = (*it).begin(); iter != (*it).end(); iter ++)
            {
                // index
                const int x = iter->first, y = iter->second;

                for(int i = 0; i < cell_points[x][y].size(); i ++)
                {
                    double px = cell_points[x][y][i].x;
                    double py = cell_points[x][y][i].y;
                    double pz = cell_points[x][y][i].z;
                    int pr = cell_points[x][y][i].ring;
                    double pi = cell_points[x][y][i].intensity;

                    // original cloud
                    cloud->points[count].x = px;
                    cloud->points[count].y = py;
                    cloud->points[count].z = pz;
                    cloud->points[count].ring = pr;
                    cloud->points[count].intensity = pi;
                    count++;
                }
            }
            cloud->points.resize(count);
            cloud_list.push_back(*cloud);
        }
        //cout << "cloud_list size: " << cloud_list.size() << endl;
    }
}
// namespace Tracking
