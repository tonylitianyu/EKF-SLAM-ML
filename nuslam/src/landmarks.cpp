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
#include "sensor_msgs/LaserScan.h"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/circle_fitting.hpp"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>


enum class LMState {INIT, WAIT, UPDATE};

static LMState state_machine = LMState::INIT;

/// \brief - method for identifying landmarks
class Landmarks{
    private:
        ros::NodeHandle n;
        ros::Timer timer;
        ros::Subscriber scan_sub;
        ros::Publisher scan_tube_pub;


        rigid2d::CircleFitting cf;
        std::vector<double> ranges_arr;
        std::vector<rigid2d::Vector2D> circle_pos;

        std::string turtle_frame_id;
    public:
        /// \brief create the initial setup for the simulator
        /// \param nh - the node handle for ROS
        /// \param turtle_frame_id - the frames for displaying scan tube is different in actual turtle and simulation
        Landmarks(ros::NodeHandle nh, std::string turtle_frame_id):
        timer(nh.createTimer(ros::Duration(0.01), &Landmarks::main_loop, this)),
        scan_sub(nh.subscribe("scan", 1000, &Landmarks::callback_scan, this)),
        scan_tube_pub(nh.advertise<visualization_msgs::MarkerArray>("scan_sensor", 10, true)),
        turtle_frame_id(turtle_frame_id)
        {

        }

        /// \brief callback function for receving laser scan reading
        /// \param point - received laser scan range data
        void callback_scan(const sensor_msgs::LaserScan &point)
        {
            state_machine = LMState::WAIT;
            ranges_arr.clear();
            
            for (int i = 0; i < 360; i++){
                
                ranges_arr.push_back(point.ranges[i]);
            }

            state_machine = LMState::UPDATE;

        }

        /// \brief visualize tubes based on laser scan points
        void show_scan_tube(){
            visualization_msgs::MarkerArray scan_tube_marker_array;

            for (int j = 0; j < circle_pos.size(); j++){

                visualization_msgs::Marker scan_tube_marker;
                scan_tube_marker.header.frame_id = turtle_frame_id;
                scan_tube_marker.header.stamp = ros::Time::now();
                scan_tube_marker.ns = "scan";
                scan_tube_marker.lifetime = ros::Duration(0.1);
                

                scan_tube_marker.id = j;
                scan_tube_marker.type = visualization_msgs::Marker::CYLINDER;
                scan_tube_marker.scale.x = 0.1;
                scan_tube_marker.scale.y = 0.1;
                scan_tube_marker.scale.z = 0.3;
                scan_tube_marker.color.r = 1.0;
                scan_tube_marker.color.g = 1.0;
                scan_tube_marker.color.a = 1.0;

                scan_tube_marker.pose.position.x = circle_pos[j].x;
                scan_tube_marker.pose.position.y = circle_pos[j].y;
                scan_tube_marker.pose.position.z = 0.15;
                scan_tube_marker.pose.orientation.w = 1.0;

                scan_tube_marker_array.markers.push_back(scan_tube_marker);

            }

            scan_tube_pub.publish(scan_tube_marker_array);
        }

        /// \brief publish laser scan frame
        void publishLaserFrame(){
            static tf2_ros::TransformBroadcaster scan_br;
            geometry_msgs::TransformStamped scan_transform;
            scan_transform.header.stamp = ros::Time::now();
            scan_transform.header.frame_id = turtle_frame_id;
            scan_transform.child_frame_id = "laser";
            scan_transform.transform.translation.x = 0.0;
            scan_transform.transform.translation.y = 0.0;
            scan_transform.transform.translation.z = 0.0;
            scan_transform.transform.rotation.x = 0.0;
            scan_transform.transform.rotation.y = 0.0;
            scan_transform.transform.rotation.z = 0.0;
            scan_transform.transform.rotation.w = 1.0;
            scan_br.sendTransform(scan_transform);
        }




        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            switch(state_machine)
            {
                case LMState::INIT:
                    cf = rigid2d::CircleFitting();
                    state_machine = LMState::WAIT;
                    break;
                case LMState::WAIT:
                    
                    break;
                case LMState::UPDATE:
                    publishLaserFrame();
                    circle_pos = cf.approxCirclePositions(ranges_arr);
                    show_scan_tube();
                    break;
                default:
                    throw std::logic_error("Invalid State");

            }

        }


        
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_turtle");
    ros::NodeHandle n;

    std::string turtle_frame_id;
    
    if (n.getParam("turtle_frame_id", turtle_frame_id))
    {
        ROS_INFO("turtle_frame_id: %s", turtle_frame_id.c_str());
    }else
    {
        ROS_ERROR("Unable to get param 'turtle_frame_id'");
    }

    Landmarks lm = Landmarks(n, turtle_frame_id);
    ros::spin();



    return 0;

}