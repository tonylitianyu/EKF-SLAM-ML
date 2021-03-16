/// \file  landmarks.cpp
/// \brief Detect landmarks from laser scan
///
/// PARAMETERS:
///     left_wheel_joint (string):  The name of the left wheel joint
///     right_wheel_joint (string):  The name of the right wheel joint
///     wheel_base (double):  The distance between the wheels
///     wheel_radius (double):  The radius of the wheels
/// PUBLISHES:
///     joint_pub (sensor_msgs::JointState): Publishes the new joint states
/// SUBSCRIBES:
///     vel_sub (geometry_msgs::Twist): Subscribes to the velocity control command



#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "rigid2d/diff_drive.hpp"


/// \brief - method for identifying landmarks
class Landmarks{
    private:
        ros::NodeHandle n;
        ros::Timer timer;

    public:
        /// \brief create the initial setup for the simulator
        ///
        Landmarks(ros::NodeHandle nh):
        timer(nh.createTimer(ros::Duration(0.01), &Landmarks::main_loop, this))
        {

        }



        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
        }


        
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_turtle");
    ros::NodeHandle n;
    

    Landmarks lm = Landmarks(n);
    ros::spin();



    return 0;

}