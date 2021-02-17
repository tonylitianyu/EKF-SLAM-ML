/// \file  follow_circle.cpp
/// \brief let the robot drive in a circle of a specified radius at a specified speed
///
/// 
/// PARAMETERS:
///     ~speed (double):  circular trajectory speed
///     ~radius (double):  circular trajectory radius
/// PUBLISHES:
///     pub (geometry_msgs::Twist): Publishes the velocity command for circular trajectory
/// SERVICES:
///     control_srv (nuturtle_robot::Control): Resets the robot to a given configuration


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nuturtle_robot/Control.h"

/// \brief control the robot to follow the circle
class FollowCircle
{
    private:
        double speed;
        double radius;
        double angular_speed;
        ros::Publisher pub;
        ros::ServiceServer control_srv;
        ros::Timer timer;
        bool start_flag;
        std::string mode;

    public:

        /// \brief create the controller for following the circle
        ///
        /// \param nh - the node handle for ROS
        /// \param speed_param - circular trajectory speed
        /// \param radius_param - circular trajectory radius
        FollowCircle(ros::NodeHandle nh, double speed_param, double radius_param):
        timer(nh.createTimer(ros::Duration(0.1), &FollowCircle::main_loop, this)),
        speed(speed_param),
        radius(radius_param),
        pub(nh.advertise<geometry_msgs::Twist>("cmd_vel", 100)),
        control_srv(nh.advertiseService("control", &FollowCircle::callback_control_service, this)),
        angular_speed(0.0),
        start_flag(false),
        mode("clockwise")
        {
            angular_speed = speed/radius;
        }

        /// \brief callback function for control service
        /// \param req - service request parameters
        /// \param res - service response
        /// \return service success
        bool callback_control_service(nuturtle_robot::Control::Request &req, nuturtle_robot::Control::Response &res)
        {
            start_flag = true;
            mode = req.mode;
            
            return true;
        }

        /// \brief publish control velocity command
        void publishCircleVel()
        {
            
            geometry_msgs::Twist vel_msg;
            if (mode == "clockwise")
            {
                vel_msg.linear.x = -speed;
                vel_msg.angular.z = -angular_speed;
            }
            else if (mode == "counter_clockwise")
            {
                vel_msg.linear.x = speed;
                vel_msg.angular.z = angular_speed;
            }
            else
            {
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
            }
            

            pub.publish(vel_msg);
            
        }

        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            if (start_flag){
                publishCircleVel();
            }
        }


};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_circle");
    ros::NodeHandle n;

    double speed;
    double radius;

    if (ros::param::get("~speed", speed))
    {
        ROS_INFO("speed: %f", speed);
    }else
    {
        ROS_ERROR("Unable to get param 'speed'");
    }

    if (ros::param::get("~radius", radius))
    {
        ROS_INFO("radius: %f", radius);
    }else
    {
        ROS_ERROR("Unable to get param 'radius'");
    }



    FollowCircle fc = FollowCircle(n, speed, radius);
    ros::spin();


    return 0;

}