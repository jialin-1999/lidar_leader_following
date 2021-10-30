#include <vector>
#include <geometry_msgs/Twist.h>

using std::vector;

class GridSpeed
{
public:
    void SpeedMessage(geometry_msgs::Twist& speed_message, int& sector, vector<double>& distance);
    void RotationSelection(geometry_msgs::Twist& speed_message, int& sector, vector<double>& distance);
};
