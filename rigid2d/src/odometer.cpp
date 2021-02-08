#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cstdio>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometer");
    
    std::string odom_frame_id;
    std::string body_frame_id;
    std::string left_wheel_joint;
    std::string right_wheel_joint;

    if (n.getParam("odom_frame_id", odom_frame_id))
    {
        ROS_INFO("odom_frame_id: %s", odom_frame_id.c_str());
    }else
    {
        ROS_ERROR("Unable to get param 'odom_frame_id'");
    }

    if (n.getParam("body_frame_id", body_frame_id))
    {
        ROS_INFO("body_frame_id: %s", body_frame_id.c_str());
    }else
    {
        ROS_ERROR("Unable to get param 'max_wdot'");
    }

    if (n.getParam("frequency", frequency_param))
    {
        ROS_INFO("frequency: %d", frequency_param);
    }else
    {
        ROS_ERROR("Unable to get param 'frequency'");
    }



    ros::spin();



    return 0;

}

