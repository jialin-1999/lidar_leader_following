// std includes
#include <iostream>
#include <string>
#include <time.h>
// my include
#include "grid_navigation/GridSpeed.hpp"

using std::cout;
using std::endl;

const double sector_dim = 360;
const double safety_threshold = 0.4;
const double linear_speed = 0.3;
const double channel_length = 2;
const int sector_width = 10;
int turn_direction = 0;

void GridSpeed::SpeedMessage(geometry_msgs::Twist& speed_message, int& sector, vector<double>& distance)
{
    clock_t start, end;
    start = clock();

    if(sector == -1 || fabs(distance[sector]) < safety_threshold || fabs(distance[180]) < safety_threshold)
    {
        if(turn_direction == 1)
        {
            speed_message.linear.x = 0;
            speed_message.angular.z = 0.5;
        }
        else if(turn_direction == -1)
        {
            speed_message.linear.x = 0;
            speed_message.angular.z = -0.5;
        }
        else
            RotationSelection(speed_message, sector, distance);
    }
    else
    {
        turn_direction = 0;
        speed_message.linear.x = linear_speed;

        double angle_speed;
        double diff = abs(sector - sector_dim / 2);
        if(diff <= 1)
            speed_message.angular.z = 0;
        else
        {
            if(sector - sector_dim / 2 < 0)
                speed_message.angular.z = - log(diff) / 10;
            else
                speed_message.angular.z = log(diff) / 10;
        }
    }

    end = clock();
    //cout << (end - start) * 1.0 / CLOCKS_PER_SEC * 1000 << " ms, speed message publish time." << endl;

    return;
}

void GridSpeed::RotationSelection(geometry_msgs::Twist& speed_message, int& sector, vector<double>& distance)
{
    cout << "+++++++++++++++++Rotation Called!+++++++++++++++++++" << endl;
    cout << sector << " " << distance[sector] << " " << distance[180] << endl;

    int lstart = -1, lend = -1, rstart = -1, rend = -1;
    int lwidth = 0, rwidth = 0;
    bool lflag = true, rflag = true;
    for(size_t i = sector_dim / 2; lflag && i != sector_dim / 4 * 3; i++)
    {
        if(distance[i] >= channel_length)
        {
            lstart = i;
            for(size_t j = i + 1; j != sector_dim / 4 * 3; j++)
            {
                if(lwidth >= sector_width)
                {
                    lend = j;
                    lflag = false;
                    break;
                }
                if(distance[i] >= channel_length)
                    lwidth++;
                else
                {
                    lstart = -1;
                    lend = -1;
                }
            }
        }
    }

    for(size_t i = sector_dim / 2; rflag && i != sector_dim / 4; i--)
    {
        if(distance[i] >= channel_length)
        {
            rstart = i;
            for(size_t j = i - 1; j != sector_dim / 4; j--)
            {
                if(rwidth > sector_width)
                {
                    rend = j;
                    rflag = false;
                    break;
                }
                if(distance[i] >= channel_length)
                    rwidth++;
                else
                {
                    rstart = -1;
                    rend = -1;
                }
            }
        }
    }

    if(lstart == -1 && rstart > 0)
    {
        speed_message.linear.x = 0;
        speed_message.angular.z = -0.5;
        turn_direction = -1;
    }
    else if(lstart > 0 && rstart == -1)
    {
        speed_message.linear.x = 0;
        speed_message.angular.z = 0.5;
        turn_direction = 1;
    }
    else if(abs(lstart - sector_dim / 2) <= abs(rstart - sector_dim / 2))
    {
        speed_message.linear.x = 0;
        speed_message.angular.z = 0.5;
        turn_direction = 1;
    }
    else
    {
        speed_message.linear.x = 0;
        speed_message.angular.z = -0.5;
        turn_direction = -1;
    }
    //cout << "turn_direction:" << turn_direction << endl;
    usleep(200);
}
