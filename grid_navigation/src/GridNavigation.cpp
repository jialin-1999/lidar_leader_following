#define PCL_NO_PRECOMPILE
// std includes
#include <time.h>
#include <utility>
// ros includes
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <signal.h>
// my includes
#include "grid_navigation/velodyne_type.hpp"
#include "grid_navigation/GridNavigation.hpp"
#include "grid_navigation/GridPlotter.hpp"
#include "grid_navigation/GridSpeed.hpp"
// pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using std::pair;
using std::make_pair;
using std::cout;
using std::endl;
using std::max;
using std::min;

// height_map
std::vector<PointXYZ> points[grid_dim][grid_dim];
int num_obs[grid_dim][grid_dim] ;
double mean_z[grid_dim][grid_dim];
double diff_z[grid_dim][grid_dim];

// primary_histogram
double primary_histogram[sector_dim];
double vector_magnitude[grid_dim][grid_dim];
const double Histogram_Low_Threshold = 1800.0;
const double Histogram_High_Threshold = 1800.0;
const double robot_size = 0.6;         // car = 0.2; dog = 0.65;
const double safety_distance = 0.5;     // car = 0.1; dog = 0.3;
double obstacle_radius = safety_distance + robot_size;

// last_binary_histogram
double last_binary_histogram[sector_dim][3];
double binary_histogram[sector_dim];
int times = 0;

// masked_histogram
double masked_histogram[sector_dim];
const double turn_radius = 0.8;        // car = 0.1; dog = 0.65;

// steering_direction
vector<int> direction_list;
const int wide_range = 10;
int target_sector;
int navigator_id = 180;

// grid_plotter
Grid_Navigation::GridPlotter grid_plotter;
visualization_msgs::MarkerArray direct_markers, obstacle_markers;
PointXYZ nearest_obstacle[sector_dim];
vector<double> yaw_list;

// target_position
PointXYZ target_position;

// speed controler
GridSpeed grid_speed;
vector<double> distance_list;
geometry_msgs::Twist speed_message;
ros::Publisher speed_publisher;
ros::Publisher dog_publisher;

vector<double> final_list;

// for calculation
double RadtoAngle(double x)
{
    return x * 180.0 / M_PI;
}
// for calculation
double AngletoRad(double x)
{
    return x / 180.0 * M_PI;
}
// for calculation
int Mod(int a, int b)
{
    a %= b;
    if(a < 0)
        a += b;
    return a;
}
// for calculation
double Gauss(double dist, double grid_dim)
{
    double t =  pow(grid_dim / 2, 2) / 10.0;
    return 1.0 * pow(M_E, -1.0 * dist / t ) * 10000;
}
// for calculation
int AngleDifference(int left, int right, int sector_dim)
{
    if(left == -1 || right == -1)
        return 0;
    int diff = left - right;
    return min(min(abs(diff), abs(diff - sector_dim)), abs(diff + sector_dim));
}

void Shutdown(int signal)
{
    speed_publisher.publish(geometry_msgs::Twist());
    ROS_INFO("Robot has stopped!");
    ros::shutdown();
}

namespace Grid_Navigation
{
    Navigation::Navigation(ros::NodeHandle node, ros::NodeHandle private_node)
    {
        ROS_INFO("Navigation constructed!");
        //srand((unsigned)time(0));
        memset(last_binary_histogram, 0, sizeof(last_binary_histogram));

        map_subscriber = node.subscribe ("height1_map", 10,
                         &Navigation::obstacles_callback, this, ros::TransportHints().tcpNoDelay(true));
        target_subscriber = node.subscribe ("target_position", 10,
                            &Navigation::positions_callback, this, ros::TransportHints().tcpNoDelay(true));

        speed_publisher = node.advertise<geometry_msgs::Twist>("smoother_cmd_vel", 1);
        dog_publisher = node.advertise<std_msgs::Float64MultiArray>("speed_control", 1);
        direct_publisher = node.advertise<visualization_msgs::MarkerArray>("direct_marker", 10);
        obstacle_publisher = node.advertise<visualization_msgs::MarkerArray>("obstacle_marker", 10);

        //ros::spin();
    }

    Navigation::~Navigation(){}

    void Navigation::positions_callback(const std_msgs::Float64MultiArray &msg)
    {
        target_position.x = msg.data[0];
        target_position.y = msg.data[1];
        double target_angle = atan2(target_position.y, target_position.x) / M_PI * 180;
        navigator_id = Mod(target_angle + 180, sector_dim);
        //cout << "target position received by navigation: "
        //     << target_position.x << " " << target_position.y << " "
        //     << navigator_id << endl;
    }

    void Navigation::obstacles_callback(const GridPointCloud::ConstPtr& cloud)
    {
        clock_t start = clock();

        memset(mean_z, 0, sizeof(mean_z));
        memset(diff_z, 0, sizeof(diff_z));
        memset(num_obs, 0, sizeof(num_obs));
        for(int i = 0; i < grid_dim; i ++)
            for(int j = 0; j < grid_dim; j ++)
                points[i][j].clear();
        yaw_list.clear();
        yaw_list.resize(0);

        for(size_t i = 0; i < cloud->cells.size(); i++)
        {
            int x = cloud->cells[i].x;
            int y = cloud->cells[i].y;

            mean_z[x][y] = cloud->cells[i].mean_z;
            diff_z[x][y] = cloud->cells[i].max_z - cloud->cells[i].min_z;;
            num_obs[x][y] = cloud->cells[i].num_obs;

            for(int j = 0; j < cloud->cells[i].p_x.size(); j ++)
            {
                PointXYZ p;
                p.x = cloud->cells[i].p_x[j];
                p.y = cloud->cells[i].p_y[j];
                p.z = cloud->cells[i].p_z[j];
                points[x][y].push_back(p);
            }
        }

        /*-------------------VFH+ Construction---------------------*/
        BuildPrimaryHistogram();

        BuildBinaryHistogram();

        BuildMaskedHistogram();

        SelectSteeringDirection();
        /*-------------------VFH+ Construction---------------------*/
/*
        cout << "-------------------nearest obstacle_markers start-------------------------" << endl;
        for(int k = 0; k < sector_dim; k ++)
            cout << k * 360.0 / sector_dim << ": "
                 << nearest_obstacle[k].x << " " << nearest_obstacle[k].y << endl;
        cout << "-------------------nearest obstacle_markers end----------------------------" << endl;
*/
        distance_list.resize(sector_dim);
        for(size_t i = 0; i != sector_dim; i ++)
        {
            if(sqrt(pow(nearest_obstacle[i].x, 2) + pow(nearest_obstacle[i].y, 2)) < 0.1)
            {
                distance_list[i] = 10;
                if(nearest_obstacle[i].x < 0)
                {
                    distance_list[i] = -1;
                }
            }
            else
                distance_list[i] = sqrt(pow(nearest_obstacle[i].x, 2) + pow(nearest_obstacle[i].y, 2));
        }
/*
        cout << "-------------------distance_list start-------------------------" << endl;
        for(int k = 0; k < sector_dim; k ++)
            cout << k * 360.0 / sector_dim << ": " << distance_list[k] << endl;
        cout << "-------------------distance_list end----------------------------" << endl;
*/
        //target_sector = -1;
        signal(SIGINT, Shutdown);
        grid_speed.SpeedMessage(speed_message, target_sector, distance_list);
        speed_publisher.publish(speed_message);

        std_msgs::Float64MultiArray dog_message;
        //cout << "********************dog_message*******************" << endl;
        if(distance_list[target_sector] < 0.8)
        {
            dog_message.data.push_back(0);
            dog_message.data.push_back(0);
            //cout << 0 << " " << 0 << endl;
        }
        else
        {
            dog_message.data.push_back(0.5);
            dog_message.data.push_back(AngletoRad(target_sector - 180));
            //cout << 0.5 << " " << AngletoRad(target_sector - 180) << endl;
        }
        dog_publisher.publish(dog_message);

        grid_plotter.DirectMarkers(direct_markers, yaw_list);
        direct_publisher.publish(direct_markers);

        grid_plotter.ObstacleMarkers(obstacle_markers, nearest_obstacle);
        obstacle_publisher.publish(obstacle_markers);
        clock_t end = clock();
        //std::cout << (end - start) * 1.0 / CLOCKS_PER_SEC * 1000 << "ms grid_navigation" << endl;
    }

    void Navigation::BuildPrimaryHistogram()
    {
        memset(vector_magnitude, 0, sizeof(vector_magnitude));
        memset(primary_histogram, 0, sizeof(primary_histogram));
        memset(nearest_obstacle, 0, sizeof(nearest_obstacle));
        for(int i = 0; i < grid_dim; i ++)
        {
            for(int j = 0; j < grid_dim; j ++)
            {
                if(num_obs[i][j] > 0)
                {
                    GridMagnitude(i, j);
                    GridPrimaryHistogram(i, j);
                }
            }
        }
        for(int i = 0; i < sector_dim / 4; i ++)
            primary_histogram[i] = Histogram_Low_Threshold;
        for(int i = sector_dim - 1; i >= sector_dim * 3.0 / 4; i --)
            primary_histogram[i] = Histogram_Low_Threshold;
/*
        cout << "-------------------primary histogram start---------------------" << endl;
        for(int k = 0; k < sector_dim; k ++)
            cout << k * 360.0 / sector_dim << ": " << primary_histogram[k] << endl;
        cout << "-------------------primary histogram end-----------------------" << endl;
*/
    }

    void Navigation::GridMagnitude(int i, int j)
    {
        double dist = pow(i - grid_dim / 2, 2) + pow(j - grid_dim / 2, 2);
        vector_magnitude[i][j] = Gauss(dist, grid_dim);
    }

    void Navigation::GridPrimaryHistogram(int i, int j)
    {
        int dist = pow(i - grid_dim / 2, 2) + pow(j - grid_dim / 2, 2);
        int obstacle_radius_cells = obstacle_radius / cell_size;

        if(obstacle_radius_cells > sqrt(dist))
        {
            ROS_ERROR("Robot has collided with objects!");
            return;
        }

        double object_theta = GridTheta(i, j);
        double diff_theta = RadtoAngle(asin(obstacle_radius_cells / sqrt(dist)));
        double lTheta = object_theta - diff_theta;
        double rTheta = object_theta + diff_theta;

        int lSector, rSector;
        if(lTheta < 0)
        {
            lTheta += 360.0;
            lSector = (int)floor(lTheta * sector_dim / 360.0);
            rSector = (int)floor(rTheta * sector_dim / 360.0);
            for(int k = lSector; k < sector_dim; k++)
            {
                if(vector_magnitude[i][j] > primary_histogram[k])
                {
                    nearest_obstacle[k].x = i * cell_size - grid_dim / 2.0 * cell_size;
                    nearest_obstacle[k].y = j * cell_size - grid_dim / 2.0 * cell_size;
                }
                primary_histogram[k] = max(primary_histogram[k], vector_magnitude[i][j]);
            }
            for(int k = 0; k <= rSector; k ++ )
            {
                if(vector_magnitude[i][j] > primary_histogram[k])
                {
                    nearest_obstacle[k].x = i * cell_size - grid_dim / 2.0 * cell_size;
                    nearest_obstacle[k].y = j * cell_size - grid_dim / 2.0 * cell_size;
                }
                primary_histogram[k] = max(primary_histogram[k], vector_magnitude[i][j]);
            }
        }
        else if(rTheta >= 360)
        {
            rTheta -= 360;
            lSector = (int)floor(lTheta * sector_dim / 360.0);
            rSector = (int)floor(rTheta * sector_dim / 360.0);
            for(int k = lSector; k < sector_dim; k ++)
            {
                if(vector_magnitude[i][j] > primary_histogram[k])
                {
                    nearest_obstacle[k].x = i * cell_size - grid_dim / 2.0 * cell_size;
                    nearest_obstacle[k].y = j * cell_size - grid_dim / 2.0 * cell_size;
                }
                primary_histogram[k] = max(primary_histogram[k], vector_magnitude[i][j]);
            }
            for(int k = 0; k <= rSector; k ++)
            {
                if(vector_magnitude[i][j] > primary_histogram[k])
                {
                    nearest_obstacle[k].x = i * cell_size - grid_dim / 2.0 * cell_size;
                    nearest_obstacle[k].y = j * cell_size - grid_dim / 2.0 * cell_size;
                }
                primary_histogram[k] = max(primary_histogram[k], vector_magnitude[i][j]);
            }
        }
        else
        {
            lSector = (int)floor(lTheta * sector_dim / 360.0);
            rSector = (int)floor(rTheta * sector_dim / 360.0);
            for(int k = lSector; k <= rSector; k ++)
            {
                if(vector_magnitude[i][j] > primary_histogram[k])
                {
                    nearest_obstacle[k].x = i * cell_size - grid_dim / 2.0 * cell_size;
                    nearest_obstacle[k].y = j * cell_size - grid_dim / 2.0 * cell_size;
                }
                primary_histogram[k] = max(primary_histogram[k], vector_magnitude[i][j]);
            }
        }
    }

    double Navigation::GridTheta(int posx, int posy)
    {
        double theta = atan2(posy - grid_dim / 2, posx - grid_dim / 2);
        theta += M_PI;
        if(2.0 * M_PI - theta < 0.001)
            theta = 0;
        return RadtoAngle(theta);
    }

    void Navigation::BuildBinaryHistogram()
    {
        memset(binary_histogram, 0, sizeof(binary_histogram));

        for(int i = 0; i < sector_dim; i ++)
        {
            int count = 0;
            for(int j = 0; j < 3; j ++)
                if(last_binary_histogram[i][times])
                    count ++;
            if(count < 3)
                binary_histogram[i] = 0;
            else if(primary_histogram[i] < Histogram_Low_Threshold)
                binary_histogram[i] = 0;
            else if(primary_histogram[i] >= Histogram_High_Threshold)
                binary_histogram[i] = 1;
            else
                binary_histogram[i] = last_binary_histogram[i][times];
        }

        times = (times + 1) % 3;

        for(int i = 0; i < sector_dim; i ++)
            if(primary_histogram[i] < Histogram_Low_Threshold)
                last_binary_histogram[i][times] = 0;
            else if(primary_histogram[i] >= Histogram_High_Threshold)
                last_binary_histogram[i][times] = 1;
/*
        cout << "---------------BinaryPolarHistogram start--------------------- " << endl;
        for(int k = 0; k < sector_dim; k ++)
            cout << k << " " << k * 360.0 / sector_dim << ": " << binary_histogram[k] << endl;
        cout << "---------------BinaryPolarHistogram end----------------------- " << endl;
*/
    }

    void Navigation::BuildMaskedHistogram()
    {
        memset(masked_histogram, 0, sizeof(masked_histogram));
        double lDrive = 360.0;
        double rDrive = 0.0;

        for(int i = 0; i < grid_dim; i ++)
        {
            for(int j = 0; j < grid_dim; j ++)
            {
                if(num_obs[i][j] > 0)
                {
                    if(!DriveLeft(i, j) || !DriveRight(i, j))
                    {
                        double theta = GridTheta(i ,j);
                        if(theta < 180.0)
                            rDrive = max(rDrive, theta);
                        else
                            lDrive = min(lDrive, theta);
                    }
                }
            }
        }
        //cout << "Masked left and right: " << lDrive << " " << rDrive << endl;
        for(int i = 0; i < sector_dim; i ++)
        {
            if(!binary_histogram[i] && i >= rDrive && i <= lDrive)
                masked_histogram[i] = 0;
            else
                masked_histogram[i] = 1;
        }
/*
        cout << "---------------MaskedPolarHistogram start--------------------- " << endl;
        for(int i = 0; i < sector_dim; i ++)
            cout << i << " " << masked_histogram[i] << endl;
        cout<<"-----------------MaskedPolarHistogram end------------------------" << endl;
*/
    }

    bool Navigation::DriveLeft(int posx, int posy)
    {
        double lx = grid_dim / 2;
        double ly = turn_radius / cell_size + grid_dim / 2;

        double dist = sqrt(pow(posx - lx, 2) + pow(posy - ly, 2));
        bool flag = dist > (turn_radius + obstacle_radius) / cell_size;

        return flag;
    }

    bool Navigation::DriveRight(int posx, int posy)
    {
        double rx = grid_dim / 2;
        double ry = -turn_radius / cell_size  + grid_dim / 2;

        double dist = sqrt(pow(posx - rx, 2) + pow(posy - ry, 2));
        bool flag = dist > (turn_radius + obstacle_radius) / cell_size;

        return flag;
    }

    void Navigation::SelectSteeringDirection()
    {
        target_sector = -1;
        SteeringDirectionList();

        int min_cost = 100000000;
        int min_sector = -1;
        for(size_t i = 0; i != direction_list.size(); i++)
        {
            int c = DirectionCost(direction_list[i]);
            if(c < min_cost)
            {
                min_cost = c;
                min_sector = direction_list[i];
            }
        }
        target_sector = min_sector;
        //cout << "********************target_sector:*******************" << endl;
        //cout << target_sector << endl;

        yaw_list.push_back(target_sector * 1.0 / 180 * M_PI);

        final_list.push_back(target_sector);
        if(final_list.size() >= 50)
        {
            cout << "+++++++++++++++++++ final target: ++++++++++++++++++++++" << endl;
            for(size_t i = 0; i < final_list.size(); i++)
            {
                cout << final_list[i] << " ";
            }
            cout << endl;
            final_list.clear();
            final_list.resize(0);
        }
    }

    void Navigation::SteeringDirectionList()
    {
        direction_list.clear();

        int start_sector = -1;
        for(int i = 0; i < sector_dim; i ++)
        {
            if(!masked_histogram[i])
            {
                start_sector = i;
                break;
            }
        }

        if(start_sector == -1)
        {
            ROS_INFO("No Direction!");
            return;
        }

        int left = start_sector;
        for(int i = start_sector; i < (start_sector + sector_dim); i ++)
        {
            if(masked_histogram[i % sector_dim])
            {
                int lSector = left;
                int rSector = i - 1;
                left = i + 1;
                //cout << i << " " << lSector << " " << rSector << endl;
                if(lSector > rSector)
                    continue;

                // more direction list;
                int midS = Mod(lSector + wide_range * 1.0 / 2, sector_dim);
                int right_sector = Mod(rSector - wide_range * 1.0 / 2, sector_dim);
                while(midS < right_sector)
                {
                    direction_list.push_back(midS);
                    midS = Mod(midS + wide_range * 1.0 / 2, sector_dim);
                }
/*
                // original direction list;
                int mid = Mod((lSector + rSector) >> 1, sector_dim);
                direction_list.push_back(mid);

                if(rSector - lSector + 1 >= wide_range)
                {
                    int midL = Mod(lSector + wide_range * 1.0 / 2, sector_dim);
                    int midR = Mod(rSector - wide_range * 1.0 / 2, sector_dim);
                    direction_list.push_back(midL);
                    direction_list.push_back(midR);
                }
*/
            }
        }

        if(left < start_sector + sector_dim)
        {
            int lSector = left;
            int rSector = start_sector + sector_dim - 1;
            if(lSector <= rSector - 1)
            {
                // more direction list;
                int midS = Mod(lSector + wide_range * 1.0 / 2, sector_dim);
                int right_sector = Mod(rSector - wide_range * 1.0 / 2, sector_dim);
                while(midS < right_sector)
                {
                    direction_list.push_back(midS);
                    midS = Mod(midS + wide_range * 1.0 / 2, sector_dim);
                }
/*
                int mid = Mod((lSector + rSector) >> 1, sector_dim);
                direction_list.push_back(mid);

                if(rSector - lSector + 1 >= wide_range)
                {
                    int midL = Mod(lSector + (wide_range >> 1), sector_dim);
                    int midR = Mod(rSector - (wide_range >> 1), sector_dim);
                    direction_list.push_back(midL);
                    direction_list.push_back(midR);
                }
*/
            }
        }
        //cout << "================direction_list start: =========================" << endl;
        for(size_t i = 0; i < direction_list.size(); i ++)
        {
            //cout << direction_list[i] << " ";
            yaw_list.push_back(direction_list[i] * 1.0 / 180 * M_PI);
        }
        //cout << endl << "=================direction_list end: =======================" << endl;
    }

    int Navigation::DirectionCost(int sector)
    {
        int param_obstacle = 0;
        int param_navigator = 1;
        return param_obstacle * AngleDifference(sector, 180, sector_dim)
             + param_navigator * AngleDifference(sector, navigator_id, sector_dim);
    }
}
