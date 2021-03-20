/// \file  unknown_data_assoc.cpp
/// \brief Extended Kalman Filter SLAM node with unknown data association
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
///     odom_path_pub (nav_msgs::Path):  Publishes the odometry path
///     slam_path_pub (nav_msgs::Path):  Publishes the slam path
///     slam_tube_pub (visualization_msgs::MarkerArray): Publishes the slam estimated tubes
/// SUBSCRIBES:
///     joint_state_sub (sensor_msgs::JointState): Subscribes to the current wheel joint angle
///     fake_sensor_sub (visualization_msgs::MarkerArray): Fake sensor measurements tubes
///     scan_sensor_sub (visualization_msgs::MarkerArray): Scan sensor measurements tubes


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
        ros::Publisher odom_path_pub;
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

        nav_msgs::Path real_path;

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
        odom_path_pub(nh.advertise<nav_msgs::Path>("odom_path", 100)),
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

            rigid2d::Twist2D body_twist_odom = dd.getBodyTwistForUpdate(delta_left_wheel_angle, delta_right_wheel_angle);
            dd.updatePose(delta_left_wheel_angle, delta_right_wheel_angle);

            static tf2_ros::TransformBroadcaster odom_br;

            tf2::Quaternion q;
            q.setRPY(0,0,dd.getTheta());

            // nav_msgs::Odometry odom;
            // odom.header.stamp = ros::Time::now();
            // odom.header.frame_id = odom_frame_id;
            // odom.pose.pose.position.x = dd.getPosition().x;
            // odom.pose.pose.position.y = dd.getPosition().y;
            // odom.pose.pose.position.z = 0.0;
            // odom.pose.pose.orientation.x = q.x();
            // odom.pose.pose.orientation.y = q.y();
            // odom.pose.pose.orientation.z = q.z();
            // odom.pose.pose.orientation.w = q.w();
            // odom.child_frame_id = body_frame_id;
            // odom.twist.twist.linear.x = body_twist_odom.linearX();
            // odom.twist.twist.linear.y = body_twist_odom.linearY();
            // odom.twist.twist.angular.z = body_twist_odom.angular();
            // odom_pub.publish(odom);

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

            //odom path
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "world";
            pose.pose.position.x = dd.getPosition().x;
            pose.pose.position.y = dd.getPosition().y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            real_path.header.stamp = ros::Time::now();
            real_path.header.frame_id = "world";
            real_path.poses.push_back(pose);

            odom_path_pub.publish(real_path);

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

        /// \brief Get the current odometry moving twist
        /// \return current moving twist
        rigid2d::Twist2D & getCurrentTwist(){
            body_twist = dd.getBodyTwistForUpdate(delta_left_wheel_angle*10.0, delta_right_wheel_angle*10.0); //odom publishes at 100Hz but slam updates at 10Hz, so times 10.
            return body_twist;
        }

        /// \brief Get the current pose of in odom
        /// \return current pose of the turtle in odom
        rigid2d::Transform2D getCurrentPose(){
            rigid2d::Vector2D trans = {dd.getPosition().x, dd.getPosition().y};
            rigid2d::Transform2D Tot = rigid2d::Transform2D(trans, dd.getTheta());
            return Tot;
        }


    
};


enum class SLAMState {INIT, UPDATE};

static SLAMState state_machine = SLAMState::INIT;

/// \brief Organizes sensor reading and update for SLAM
class SLAM
{

    private:
        ros::Timer timer;
        ros::Subscriber fake_sensor_sub;
        ros::Subscriber scan_sensor_sub;
        ros::Publisher slam_path_pub;
        ros::Publisher slam_tube_pub;
        nav_msgs::Path slam_path;

        std::string odom_frame_id;
        std::string body_frame_id;
        double wheel_base;
        double wheel_radius;

        Odometer & odometer;
        rigid2d::EKF_SLAM slam_agent;
        

        //measurement model
        mat sensor_reading;



        int max_n_tubes;


        bool sensor_update_flag;
        bool state_update_flag;

        std::vector<bool> visible_list;
        std::vector<bool> known_list;

        std::vector<rigid2d::Vector2D> scan_measures;

    public:
        /// \brief create the initial setup for slam
        ///
        /// \param nh - the node handle for ROS
        /// \param odom_frame_id_str - The name of the odometry tf frame
        /// \param body_frame_id_str - The name of the body tf frame
        /// \param wheel_base_val - The distance between the wheels 
        /// \param wheel_radius_val - The radius of the wheels
        /// \param odometer - The odometer object for getting the current twist
        SLAM(ros::NodeHandle nh, std::string odom_frame_id_str, std::string body_frame_id_str, double wheel_base_val, double wheel_radius_val, Odometer & odometer):
        timer(nh.createTimer(ros::Duration(0.1), &SLAM::main_loop, this)),
        scan_sensor_sub(nh.subscribe("scan_sensor", 1000, &SLAM::callback_scan_sensor, this)),
        slam_path_pub(nh.advertise<nav_msgs::Path>("slam_path", 100)),
        slam_tube_pub(nh.advertise<visualization_msgs::MarkerArray>("slam_tube", 10, true)),
        odom_frame_id(odom_frame_id_str),
        body_frame_id(body_frame_id_str),
        wheel_base(wheel_base_val),
        wheel_radius(wheel_radius_val),
        odometer(odometer),
        max_n_tubes(20),
        sensor_update_flag(false),
        state_update_flag(false)
        {
            for (int i = 0; i < max_n_tubes; i++){
                visible_list.push_back(false);  //for measurement
                known_list.push_back(false);    //for slam visual
            }

            sensor_reading = zeros<mat>(max_n_tubes*2,1);
        }

        /// \brief publish necessary frames
        void publishFrames(){
            static tf2_ros::TransformBroadcaster world_map_br;

            geometry_msgs::TransformStamped world_map_transform;
            world_map_transform.header.stamp = ros::Time::now();
            world_map_transform.header.frame_id = "world";
            world_map_transform.child_frame_id = "map";
            world_map_transform.transform.rotation.w = 1.0;
            world_map_br.sendTransform(world_map_transform);


            static tf2_ros::TransformBroadcaster map_odom_br;

            //calulcate odom in map
            rigid2d::Transform2D Tto = odometer.getCurrentPose().inv();
            rigid2d::Vector2D slam_trans = {slam_agent.getStateX(), slam_agent.getStateY()};
            rigid2d::Transform2D Tmt_temp = rigid2d::Transform2D(slam_trans, slam_agent.getStateTheta());
            Tmt_temp*=Tto;
            rigid2d::Transform2D Tmt = Tmt_temp;
            tf2::Quaternion odom_ori;
            odom_ori.setRPY(0,0,Tmt.theta());



            geometry_msgs::TransformStamped map_odom_transform;
            map_odom_transform.header.stamp = ros::Time::now();
            map_odom_transform.header.frame_id = "map";
            map_odom_transform.child_frame_id = odom_frame_id;
            map_odom_transform.transform.translation.x = Tmt.x();
            map_odom_transform.transform.translation.y = Tmt.y();
            map_odom_transform.transform.rotation.x = odom_ori.x();
            map_odom_transform.transform.rotation.y = odom_ori.y();
            map_odom_transform.transform.rotation.z = odom_ori.z();

            map_odom_transform.transform.rotation.w = odom_ori.w();
            map_odom_br.sendTransform(map_odom_transform);

        }


        /// \brief callback function for receving scan sensor reading
        /// \param tube - received scan sensor marker array
        void callback_scan_sensor(const visualization_msgs::MarkerArray &tube){
            std::vector<visualization_msgs::Marker> scan_tubes = tube.markers;
            scan_measures.clear();
            
            for(int i = 0; i < scan_tubes.size(); i++){
                rigid2d::Vector2D pos = {scan_tubes[i].pose.position.x, scan_tubes[i].pose.position.y};
                scan_measures.push_back(pos);
            }

            sensor_update_flag = true;

        }

        /// \brief publish path for turtle in SLAM state
        void publishSLAMPath(){
            //add path
            tf2::Quaternion slam_ori;
            slam_ori.setRPY(0,0,slam_agent.getStateTheta());
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
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


            static tf2_ros::TransformBroadcaster slam_br;
            geometry_msgs::TransformStamped slam_transform;
            slam_transform.header.stamp = ros::Time::now();
            slam_transform.header.frame_id = "map";
            slam_transform.child_frame_id = body_frame_id;
            slam_transform.transform.translation.x = slam_agent.getStateX();
            slam_transform.transform.translation.y = slam_agent.getStateY();
            slam_transform.transform.translation.z = 0.0;
            slam_transform.transform.rotation.x = slam_ori.x();
            slam_transform.transform.rotation.y = slam_ori.y();
            slam_transform.transform.rotation.z = slam_ori.z();
            slam_transform.transform.rotation.w = slam_ori.w();
            slam_br.sendTransform(slam_transform);

        }

        /// \brief publish landmark SLAM state
        void publishSLAMLandmark(){
            mat landmark_state = slam_agent.getStateLandmark();
            visualization_msgs::MarkerArray slam_tube_marker_array; 

            for (int i = 0; i < known_list.size(); i++){
                visualization_msgs::Marker slam_tube_marker;
                slam_tube_marker.header.frame_id = "map";
                slam_tube_marker.header.stamp = ros::Time::now();
                slam_tube_marker.ns = "landmark";
                if (known_list[i] == true){
                    slam_tube_marker.action = visualization_msgs::Marker::ADD;
                }else{
                    slam_tube_marker.action = visualization_msgs::Marker::DELETE;
                }


                slam_tube_marker.lifetime = ros::Duration(0.1);

                slam_tube_marker.id = i;
                slam_tube_marker.type = visualization_msgs::Marker::CYLINDER;
                slam_tube_marker.scale.x = 0.1;
                slam_tube_marker.scale.y = 0.1;
                slam_tube_marker.scale.z = 0.3;
                slam_tube_marker.color.b = 1.0;
                slam_tube_marker.color.a = 1.0;

                slam_tube_marker.pose.position.x = landmark_state(2*i,0);
                slam_tube_marker.pose.position.y = landmark_state(2*i+1,0);
                slam_tube_marker.pose.position.z = 0.15;
                slam_tube_marker.pose.orientation.w = 1.0;

                slam_tube_marker_array.markers.push_back(slam_tube_marker);

            }

            slam_tube_pub.publish(slam_tube_marker_array);
        }


        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){

            //publishFrames();

            switch(state_machine)
            {
                case SLAMState::INIT:
                    slam_agent = rigid2d::EKF_SLAM(max_n_tubes);
                    state_machine = SLAMState::UPDATE;
                    break;
                case SLAMState::UPDATE:
                    if (sensor_update_flag){
                        slam_agent.prediction(odometer.getCurrentTwist());
                        slam_agent.data_association(scan_measures, known_list); //update known list
                        sensor_update_flag = false;
                        state_update_flag = true;
                    }
                    publishFrames();
                    publishSLAMPath();
                    publishSLAMLandmark();

                    break;
                default:
                    throw std::logic_error("Invalid State");

            }

        }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "unknown_data_assoc");
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
    SLAM slam_obj = SLAM(n, odom_frame_id, body_frame_id, wheel_base, wheel_radius, odo_obj);
    ros::spin();



    return 0;

}