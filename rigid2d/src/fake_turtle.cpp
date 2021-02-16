/// \file  fake_turtle.cpp
/// \brief Creates kinematic simulation of a diff drive robot
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


/// \brief - the simulation class for diff drive turtle robot
class FakeTurtle{
    private:
        ros::NodeHandle n;
        ros::Subscriber vel_sub;
        ros::Publisher joint_pub;
        ros::Timer timer;

        rigid2d::DiffDrive dd;
        std::string left_wheel_joint;
        std::string right_wheel_joint;
        double wheel_base;
        double wheel_radius;
        double left_wheel_angle;
        double right_wheel_angle;

        double x_vel;
        double y_vel;
        double ang_vel;

    public:
        /// \brief create the initial setup for the simulator
        ///
        /// \param nh - the node handle for ROS
        /// \param left_wheel_joint_str - The name of the left wheel joint
        /// \param right_wheel_joint_str - The name of the right wheel joint
        /// \param wheel_base_val - The distance between the wheels 
        /// \param wheel_radius_val - The radius of the wheels
        FakeTurtle(ros::NodeHandle nh, std::string left_wheel_joint_str, std::string right_wheel_joint_str, double wheel_base_val, double wheel_radius_val):
        timer(nh.createTimer(ros::Duration(0.01), &FakeTurtle::main_loop, this)),
        vel_sub(nh.subscribe("cmd_vel", 1000, &FakeTurtle::callback_vel, this)),
        joint_pub(nh.advertise<sensor_msgs::JointState>("joint_states", 100)),
        dd(rigid2d::DiffDrive(wheel_base_val, wheel_radius_val)),
        left_wheel_angle(0.0),
        right_wheel_angle(0.0),
        left_wheel_joint(left_wheel_joint_str),
        right_wheel_joint(right_wheel_joint_str)
        {

        }

        /// \brief callback function for velocity command
        /// \param vel - velocity command
        void callback_vel(const geometry_msgs::Twist &vel)
        {
            x_vel = vel.linear.x;
            y_vel = vel.linear.y;
            ang_vel = vel.angular.z;

        }

        void publishJointState(){
            rigid2d::Vector2D lin = {x_vel,y_vel};
            rigid2d::Twist2D ts = rigid2d::Twist2D(ang_vel, lin);
            rigid2d::Vector2D wheel_vel = dd.calculateWheelVelocity(ts);


            double delta_wheel_left = (wheel_vel.x/100.0);   //100.0 is the timer frequency in odometer
            double delta_wheel_right = (wheel_vel.y/100.0);

            left_wheel_angle += delta_wheel_left;  
            right_wheel_angle += delta_wheel_right;


            sensor_msgs::JointState joint_msg;
            joint_msg.header.stamp = ros::Time::now();
            joint_msg.name.push_back(left_wheel_joint);
            joint_msg.name.push_back(right_wheel_joint);

            joint_msg.position.push_back(left_wheel_angle);
            joint_msg.position.push_back(right_wheel_angle);
            joint_msg.velocity.push_back(delta_wheel_left);
            joint_msg.velocity.push_back(delta_wheel_right);
            joint_pub.publish(joint_msg);
        }


        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            publishJointState();
        }


        
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_turtle");
    ros::NodeHandle n;
    
    std::string left_wheel_joint;
    std::string right_wheel_joint;
    double wheel_base;
    double wheel_radius;

    if (n.getParam("left_wheel_joint", left_wheel_joint))
    {
        ROS_INFO("left_wheel_joint: %s", left_wheel_joint.c_str());
    }else
    {
        ROS_ERROR("Unable to get param 'left_wheel_joint'");
    }

    if (n.getParam("right_wheel_joint", right_wheel_joint))
    {
        ROS_INFO("right_wheel_joint: %s", right_wheel_joint.c_str());
    }else
    {
        ROS_ERROR("Unable to get param 'right_wheel_joint'");
    }

    if (n.getParam("wheel_base", wheel_base))
    {
        ROS_INFO("wheel_base: %f", wheel_base);
    }else
    {
        ROS_ERROR("Unable to get param 'wheel_base'");
    }

    if (n.getParam("wheel_radius", wheel_radius))
    {
        ROS_INFO("wheel_radius: %f", wheel_radius);
    }else
    {
        ROS_ERROR("Unable to get param 'wheel_radius'");
    }


    FakeTurtle ft = FakeTurtle(n, left_wheel_joint, right_wheel_joint, wheel_base, wheel_radius);
    ros::spin();



    return 0;

}