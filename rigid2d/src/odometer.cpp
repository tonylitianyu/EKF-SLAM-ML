/// \file  odometer.cpp
/// \brief Estimates the configuration of the robot based on information about how the wheels have moved
///
/// PARAMETERS:
///     odom_frame_id (string): The name of the odometry tf frame
///     body_frame_id (string): The name of the body tf frame
///     left_wheel_joint (string):  The name of the left wheel joint
///     right_wheel_joint (string):  The name of the right wheel joint
///     wheel_base (double):  The distance between the wheels
///     wheel_radius (double):  The radius of the wheels
/// PUBLISHES:
///     odom_pub (nav_msgs::Odometry): Publishes the robot updated odometry
/// SUBSCRIBES:
///     joint_state_sub (sensor_msgs::JointState): Subscribes to the current wheel joint angle
/// SERVICES:
///     set_pose_srv (rigid2d::SetPose): Resets the robot to a given configuration


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "rigid2d/diff_drive.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rigid2d/SetPose.h"
#include <cstdio>


/// \brief keep track of diff drive robot odometry
class Odometer
{
    private:
        ros::NodeHandle n;
        ros::Subscriber joint_state_sub;
        ros::Publisher odom_pub;
        ros::ServiceServer set_pose_srv;
        ros::Timer timer;

        rigid2d::DiffDrive dd;
        std::string odom_frame_id;
        std::string body_frame_id;
        double wheel_base;
        double wheel_radius;

        double prev_left_wheel_angle;
        double prev_right_wheel_angle;
        double left_wheel_angle;
        double right_wheel_angle;

    public:

        /// \brief create the initial setup for odometer
        ///
        /// \param nh - the node handle for ROS
        /// \param odom_frame_id_str - The name of the odometry tf frame
        /// \param body_frame_id_str - The name of the body tf frame
        /// \param wheel_base_val - The distance between the wheels 
        /// \param wheel_radius_val - The radius of the wheels
        Odometer(ros::NodeHandle nh, std::string odom_frame_id_str, std::string body_frame_id_str, double wheel_base_val, double wheel_radius_val):
        timer(nh.createTimer(ros::Duration(0.1), &Odometer::main_loop, this)),
        joint_state_sub(nh.subscribe("joint_states", 1000, &Odometer::callback_joints, this)),
        odom_pub(nh.advertise<nav_msgs::Odometry>("odom", 100)),
        dd(rigid2d::DiffDrive(wheel_base_val, wheel_radius_val)),
        set_pose_srv(nh.advertiseService("set_pose", &Odometer::callback_set_pose_service, this)),
        odom_frame_id(odom_frame_id_str),
        body_frame_id(body_frame_id_str),
        wheel_base(wheel_base_val),
        wheel_radius(wheel_radius_val),
        left_wheel_angle(0.0),
        right_wheel_angle(0.0),
        prev_left_wheel_angle(0.0),
        prev_right_wheel_angle(0.0)
        {
        }

        /// \brief publish updated odometry information and transformation
        void publishUpdatedOdometry(){

            double delta_left_wheel_angle = left_wheel_angle - prev_left_wheel_angle;
            double delta_right_wheel_angle = right_wheel_angle - prev_right_wheel_angle;
            rigid2d::Twist2D body_twist = dd.getBodyTwistForUpdate(delta_left_wheel_angle, delta_right_wheel_angle);
            dd.updatePose(delta_left_wheel_angle, delta_right_wheel_angle);

            static tf2_ros::TransformBroadcaster odom_br;

            tf2::Quaternion q;
            q.setRPY(0,0,dd.getTheta());

            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = odom_frame_id;
            odom.pose.pose.position.x = dd.getPosition().x;
            odom.pose.pose.position.y = dd.getPosition().y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();
            odom.child_frame_id = body_frame_id;
            odom.twist.twist.linear.x = body_twist.linearX();
            odom.twist.twist.linear.y = body_twist.linearY();
            odom.twist.twist.angular.z = body_twist.angular();
            odom_pub.publish(odom);

            //source: http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29
            geometry_msgs::TransformStamped odom_transform;
            odom_transform.header.stamp = ros::Time::now();
            odom_transform.header.frame_id = odom_frame_id;
            odom_transform.child_frame_id = body_frame_id;
            odom_transform.transform.translation.x = dd.getPosition().x;
            odom_transform.transform.translation.y = dd.getPosition().y;
            odom_transform.transform.translation.z = 0.0;
            odom_transform.transform.rotation.x = q.x();
            odom_transform.transform.rotation.y = q.y();
            odom_transform.transform.rotation.z = q.z();
            odom_transform.transform.rotation.w = q.w();
            odom_br.sendTransform(odom_transform);

        }

        /// \brief callback function for set_pose service
        /// \param req - service request parameters
        /// \param res - service response
        /// \return service success
        bool callback_set_pose_service(rigid2d::SetPose::Request &req, rigid2d::SetPose::Response &res)
        {
            
            double x = req.x;
            double y = req.y;
            double theta = req.theta;

            rigid2d::Vector2D set_tran = rigid2d::Vector2D(x,y);
            dd = rigid2d::DiffDrive(wheel_base, wheel_radius, set_tran, theta);
            
            return true;
        }


        /// \brief callback function for joint state subscriber
        /// \param joints - new wheel angle states
        void callback_joints(const sensor_msgs::JointState &joints)
        {

            prev_left_wheel_angle = left_wheel_angle;
            prev_right_wheel_angle = right_wheel_angle;
            left_wheel_angle = joints.position[0];
            right_wheel_angle = joints.position[1];

        }
        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            publishUpdatedOdometry();
        }


    
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometer");
    ros::NodeHandle n;
    
    std::string odom_frame_id;
    std::string body_frame_id;
    std::string left_wheel_joint;
    std::string right_wheel_joint;
    double wheel_base;
    double wheel_radius;

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
        ROS_ERROR("Unable to get param 'body_frame_id'");
    }

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


    Odometer odo_obj = Odometer(n, odom_frame_id, body_frame_id, wheel_base, wheel_radius);
    ros::spin();



    return 0;

}

