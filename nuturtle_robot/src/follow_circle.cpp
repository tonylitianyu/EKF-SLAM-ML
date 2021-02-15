
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_circle");
    ros::NodeHandle n;

    ros::spin();


    return 0;

}