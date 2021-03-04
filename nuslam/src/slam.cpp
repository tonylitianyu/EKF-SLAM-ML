/// \file  slam.cpp
/// \brief Extended Kalman Filter SLAM algorithm
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
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/ekf_slam.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rigid2d/SetPose.h"

#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Path.h"

#include <cstdio>
#include <armadillo>
#include <random>

using namespace arma;

/// \brief keep track of diff drive robot odometry
class Odometer
{
    private:
        ros::NodeHandle n;
        ros::Subscriber joint_state_sub;
        ros::Publisher odom_pub;
        ros::Timer timer;

        rigid2d::DiffDrive dd;
        rigid2d::Twist2D body_twist;
        std::string odom_frame_id;
        std::string body_frame_id;
        double wheel_base;
        double wheel_radius;

        double delta_left_wheel_angle;
        double delta_right_wheel_angle;
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
        timer(nh.createTimer(ros::Duration(0.01), &Odometer::main_loop, this)),
        joint_state_sub(nh.subscribe("joint_states", 1000, &Odometer::callback_joints, this)),
        odom_pub(nh.advertise<nav_msgs::Odometry>("odom", 100)),
        dd(rigid2d::DiffDrive(wheel_base_val, wheel_radius_val)),
        odom_frame_id(odom_frame_id_str),
        body_frame_id(body_frame_id_str),
        wheel_base(wheel_base_val),
        wheel_radius(wheel_radius_val),
        left_wheel_angle(0.0),
        right_wheel_angle(0.0),
        delta_left_wheel_angle(0.0),
        delta_right_wheel_angle(0.0)
        {
        }

        /// \brief publish updated odometry information and transformation
        void publishUpdatedOdometry(){

            body_twist = dd.getBodyTwistForUpdate(delta_left_wheel_angle, delta_right_wheel_angle);
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



        /// \brief callback function for joint state subscriber
        /// \param joints - new wheel angle states
        void callback_joints(const sensor_msgs::JointState &joints)
        {

            left_wheel_angle = joints.position[0];
            right_wheel_angle = joints.position[1];
            delta_left_wheel_angle = joints.velocity[0];
            delta_right_wheel_angle = joints.velocity[1];
            
        }
        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            publishUpdatedOdometry();
        }


        rigid2d::Twist2D & getCurrentTwist(){
            return body_twist;
        }


    
};


class SLAM
{

    private:
        ros::Timer timer;
        ros::Subscriber fake_sensor_sub;
        ros::Publisher slam_path_pub;
        nav_msgs::Path slam_path;

        std::string odom_frame_id;
        double wheel_base;
        double wheel_radius;

        Odometer & odometer;
        rigid2d::EKF_SLAM slam_agent;
        

        //measurement model
        mat sensor_reading;


        double vx_std;
        double the_std;
        double covar_sensor_x;
        double covar_sensor_y;
        double max_visible_dis;

        int state_machine;
        int n_tubes;



    public:
        SLAM(ros::NodeHandle nh, std::string odom_frame_id_str, 
                double wheel_base_val, double wheel_radius_val, Odometer & odometer, 
                double vx_std, double the_std, double covar_sensor_x, double covar_sensor_y, double max_visible_dis):
        timer(nh.createTimer(ros::Duration(0.01), &SLAM::main_loop, this)),
        fake_sensor_sub(nh.subscribe("fake_sensor", 1000, &SLAM::callback_fake_sensor, this)),
        slam_path_pub(nh.advertise<nav_msgs::Path>("slam_path", 100)),
        odom_frame_id(odom_frame_id_str),
        wheel_base(wheel_base_val),
        wheel_radius(wheel_radius_val),
        odometer(odometer),
        vx_std(vx_std),
        the_std(the_std),
        covar_sensor_x(covar_sensor_x),
        covar_sensor_y(covar_sensor_y),
        max_visible_dis(max_visible_dis),
        n_tubes(0)
        {

            state_machine = 0;
        }

        void publishFrames(){
            static tf2_ros::TransformBroadcaster world_map_br;

            geometry_msgs::TransformStamped world_map_transform;
            world_map_transform.header.stamp = ros::Time::now();
            world_map_transform.header.frame_id = "world";
            world_map_transform.child_frame_id = "map";
            world_map_transform.transform.rotation.w = 1.0;
            world_map_br.sendTransform(world_map_transform);


            static tf2_ros::TransformBroadcaster map_odom_br;

            geometry_msgs::TransformStamped map_odom_transform;
            map_odom_transform.header.stamp = ros::Time::now();
            map_odom_transform.header.frame_id = "map";
            map_odom_transform.child_frame_id = odom_frame_id;
            map_odom_transform.transform.rotation.w = 1.0;
            map_odom_br.sendTransform(map_odom_transform);

        }



        void callback_fake_sensor(const visualization_msgs::MarkerArray &tube){
            std::vector<visualization_msgs::Marker> fake_tubes = tube.markers;
            sensor_reading = zeros<mat>(fake_tubes.size()*2,1);
            for (int i = 0; i < fake_tubes.size(); i++){
                sensor_reading(i*2,0) = fake_tubes[i].pose.position.x;
                sensor_reading(i*2+1,0) = fake_tubes[i].pose.position.y;

            }

            if (fake_tubes.size() != 0 && state_machine == 0){
                state_machine = 1;
                n_tubes = fake_tubes.size();
            }



        }


        void publishSLAMPath(){
            //add path
            tf2::Quaternion slam_ori;
            slam_ori.setRPY(0,0,slam_agent.getStateTheta());
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "world";
            pose.pose.position.x = slam_agent.getStateX();
            pose.pose.position.y = slam_agent.getStateY();
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = slam_ori.x();
            pose.pose.orientation.y = slam_ori.y();
            pose.pose.orientation.z = slam_ori.z();
            pose.pose.orientation.w = slam_ori.w();

            slam_path.header.stamp = ros::Time::now();
            slam_path.header.frame_id = "world";
            slam_path.poses.push_back(pose);

            slam_path_pub.publish(slam_path);
        }


        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){

            publishFrames();


            switch(state_machine)
            {
                case 0:
                    break;
                case 1:
                    slam_agent = rigid2d::EKF_SLAM(n_tubes, {the_std, vx_std, 0.0});
                    state_machine = 2;
                    break;
                case 2:

                    slam_agent.prediction(odometer.getCurrentTwist());
                    
                    slam_agent.measurement(sensor_reading);

                    publishSLAMPath();

                    break;
                default:
                    throw std::logic_error("Invalid State");

            }

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



    double vx_std;
    double the_std;
    double covar_sensor_x;
    double covar_sensor_y;
    double max_visible_dis;

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


    if (n.getParam("vx_std", vx_std))
    {
        ROS_INFO("vx_std: %f", vx_std);
    }else
    {
        ROS_ERROR("Unable to get param 'vx_std'");
    }


    if (n.getParam("the_std", the_std))
    {
        ROS_INFO("the_std: %f", the_std);
    }else
    {
        ROS_ERROR("Unable to get param 'the_std'");
    }

    if (n.getParam("covar_sensor_x", covar_sensor_x))
    {
        ROS_INFO("covar_sensor_x: %f", covar_sensor_x);
    }else
    {
        ROS_ERROR("Unable to get param 'covar_sensor_x'");
    }

    if (n.getParam("covar_sensor_y", covar_sensor_y))
    {
        ROS_INFO("covar_sensor_y: %f", covar_sensor_y);
    }else
    {
        ROS_ERROR("Unable to get param 'covar_sensor_y'");
    }

    if (n.getParam("max_visible_dis", max_visible_dis))
    {
        ROS_INFO("max_visible_dis: %f", max_visible_dis);
    }else
    {
        ROS_ERROR("Unable to get param 'max_visible_dis'");
    }


    Odometer odo_obj = Odometer(n, odom_frame_id, body_frame_id, wheel_base, wheel_radius);
    SLAM slam_obj = SLAM(n, odom_frame_id, wheel_base, wheel_radius, odo_obj, 
                            vx_std, the_std, covar_sensor_x, covar_sensor_y, max_visible_dis);
    ros::spin();



    return 0;

}

