#ifndef GRID_CLUSTER_H_
#define GRID_CLUSTER_H_

// std includes
#include <iostream>
#include <vector>
// my includes
#include "grid_tracking/velodyne_type.hpp"
#include "grid_tracking/grid_size.hpp"

using std::vector;
using std::pair;

const int dir_num = 8;
const int dir_pair[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                            {1, 1}, {1, -1}, {-1,-1}, {-1, 1}};

namespace Tracking
{
class GridCluster
{
private:
    int radius, left, right;
    bool visit[grid_dim][grid_dim];            //visit[x][y] = 1 if (x,y) has been visited
    vector< pair<int, int> > cluster_current;  //current connected grids

public:
    GridCluster();
    ~GridCluster();

    void GenerateGrids(int num_obs[][grid_dim], int cluster_id[][grid_dim],
         vector< vector<pair<int,int> > >& cluster_list);
    void DepthFirstSearch(int num_obs[][grid_dim], int &x, int &y,
         int &cluster_count, int cluster_id[][grid_dim]);
    void GenerateClouds(vector< vector<pair<int,int> > >& cluster_list,
         vector<VPoint> cell_points[][grid_dim], vector<VPointCloud>& cloud);
};

}/*namespace Tracking*/

#endif
