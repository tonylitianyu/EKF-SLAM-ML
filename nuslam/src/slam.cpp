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
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "rigid2d/diff_drive.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rigid2d/SetPose.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
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


        mat getCurrentTwist(){
            //dtheta
            //dx
            //dy

            mat ts = zeros<mat>(3,1);
            ts(0,0) = body_twist.angular();
            ts(1,0) = body_twist.linearX();
            ts(2,0) = body_twist.linearY();
            return ts;
        }


    
};


class SLAM
{

    private:
        ros::Timer timer;

        std::string odom_frame_id;
        double wheel_base;
        double wheel_radius;

        Odometer & odometer;
        
        int n;

        //motion model
        mat state;
        mat wt; //process noise for robot
        mat sigma;
        mat Q;
        mat L;

        //measurement model
        mat R;
        mat sensor_reading;


        double vx_std;
        double the_std;

        ros::Subscriber fake_sensor_sub;

        int state_machine;

    public:
        SLAM(ros::NodeHandle nh, std::string odom_frame_id_str, 
                double wheel_base_val, double wheel_radius_val, Odometer & odometer, double vx_std, double the_std):
        timer(nh.createTimer(ros::Duration(0.01), &SLAM::main_loop, this)),
        fake_sensor_sub(nh.subscribe("fake_sensor", 1000, &SLAM::callback_fake_sensor, this)),
        odom_frame_id(odom_frame_id_str),
        wheel_base(wheel_base_val),
        wheel_radius(wheel_radius_val),
        odometer(odometer),
        vx_std(vx_std),
        the_std(the_std)
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


        std::mt19937 & get_random(){
            static std::random_device rd{};
            static std::mt19937 mt{rd()};
            return mt;
        }


        void landmark_init(){
            for (int i = 3; i < 3+2*n;){
                double coor_x = sensor_reading(i-3, 0);
                double coor_y = sensor_reading(i-2, 0);

                double r = sqrt(pow(coor_x - state(1,0),2)+pow(coor_y - state(2,0),2));
                double phi = atan2(coor_y - state(2,0), coor_x - state(1,0));

                //mx
                state(i,0) = state(1,0) + r*cos(phi+state(0,0));

                //my
                state(i+1,0) = state(2,0) + r*sin(phi+state(0,0));

                i += 2;
            }


        }


        //state EKF_SLAM(odometry, sensor)
        void EKF_SLAM_init(){


            n = 2;
            state = zeros<mat>(3+2*n,1);
            wt = zeros<mat>(3,1);

            mat sigma_top_left = zeros<mat>(3,3);
            mat sigma_top_right = zeros<mat>(3,2*n);
            mat sigma_bot_left = zeros<mat>(2*n,3);
            mat sigma_bot_right = ones<mat>(2*n,2*n);
            sigma_bot_right.eye(2*n, 2*n);
            sigma_bot_right = sigma_bot_right*10000;

            mat sigma_top = join_horiz(sigma_top_left,sigma_top_right);
            mat sigma_bot = join_horiz(sigma_bot_left,sigma_bot_right);
            sigma = join_vert(sigma_top, sigma_bot);



            Q = zeros<mat>(3+2*n, 3+2*n);
            Q(0,0) = the_std;
            Q(1,1) = vx_std;
            Q(2,2) = 0.0;


            //initialize landmark
            landmark_init();
            
        }


        void EKF_SLAM_prediction(){
            //if ddelta = 0
            mat delta = odometer.getCurrentTwist();
            mat update = zeros<mat>(3+2*n,1);
            mat At;
            mat IA;


            mat process_noise_tube = zeros<mat>(2*n,1);
            mat process_noise_combined = join_vert(wt, process_noise_tube);

            double dtheta = delta(0,0);
            double dx = delta(1,0);
            double dy = delta(2,0);

            double theta = state(0,0);
            double x = state(1,0);
            double y = state(2,0);


            if (fabs(dtheta) < 0.0001){
                //update state
                update(1,0) = dx*cos(theta);
                update(2,0) = dx*sin(theta);

                //update uncertainty
                mat A = zeros<mat>(3+2*n,3+2*n);
                A(1,0) = -dx*sin(theta);
                A(2,0) = dx*cos(theta);
                At = IA.eye(size(A)) + A;


            }else{
                update(0,0) = dtheta;
                update(1,0) = -(dx/dtheta)*sin(theta) + (dx/dtheta)*sin(theta+dtheta);
                update(2,0) = (dx/dtheta)*cos(theta) - (dx/dtheta)*cos(theta+dtheta);


                mat A = zeros<mat>(3+2*n, 3+2*n);
                A(1,0) = -(dx/dtheta)*cos(theta) + (dx/dtheta)*cos(theta+dtheta);
                A(2,0) = -(dx/dtheta)*sin(theta) + (dx/dtheta)*sin(theta+dtheta);
                At = IA.eye(size(A)) + A;

            }


            mat next_state = state + update + process_noise_combined;

            state = next_state;



            mat next_sigma = At*sigma*At.t() + Q;

            sigma = next_sigma;

        }


        void EKF_SLAM_correction(){
            double theta = state(0,0);
            double x = state(1,0);
            double y = state(2,0);

            //////////////////////
            //////////////////////

            mat H = zeros<mat>(2*n, 3+2*n);
            mat z_hat = zeros<mat>(2*n, 1);
            mat z_sensor = zeros<mat>(2*n, 1);

            std::cout << state << std::endl;

            for (int i = 0; i < 2*n;){
                double delta_x = state(i+3,0) - x;
                double delta_y = state(i+4,0) - y;
                double d = pow(delta_x,2) + pow(delta_y,2);
                

                //expected measurement
                //rj
                z_hat(i,0) = sqrt(d);
                //phij
                z_hat(i+1,0) = atan2(delta_y, delta_x) - theta;

                mat Hj;
                mat Hj_left = {{0,-delta_x/sqrt(d), -delta_y/sqrt(d)}, {-1, delta_y/d, -delta_x/d}};
                mat Hj_mid_left = zeros<mat>(2, 2*((i/2)+1-1));
                mat Hj_mid_right = {{delta_x/sqrt(d), delta_y/sqrt(d)},{-delta_y/d, delta_x/d}};
                mat Hj_right = zeros<mat>(2, 2*n - 2*((i/2)+1));
                Hj = join_horiz(Hj_left, Hj_mid_left);
                Hj = join_horiz(Hj, Hj_mid_right);
                Hj = join_horiz(Hj, Hj_right);

                H(span(i,i+1),span(0, 3+2*n-1)) = Hj;



                //z sensor
                double delta_x_sensor = sensor_reading(i,0) - x;
                double delta_y_sensor = sensor_reading(i+1,0) - y;
                double d_sensor = pow(delta_x_sensor,2) + pow(delta_y_sensor,2);
                z_sensor(i,0) = sqrt(d_sensor);
                z_sensor(i+1,0) = atan2(delta_y_sensor, delta_x_sensor) - theta;


                i +=2;

            }

            mat R = eye(2*n, 2*n)*0.01;

            mat Ki = sigma*H.t()*inv(H*sigma*H.t()+R);


            mat next_state = state + Ki*(z_sensor - z_hat);
            state = next_state;


            mat kh = Ki*H;
            mat e;
            e.eye( size(kh) );
            mat next_sigma = (e - kh)*sigma;
            sigma = next_sigma;

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
            }



        }


        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){

            publishFrames();

            switch(state_machine)
            {
                case 0:
                    break;
                case 1:
                    EKF_SLAM_init();
                    state_machine = 2;
                    break;
                case 2:
                    EKF_SLAM_prediction();
                    EKF_SLAM_correction();
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


    Odometer odo_obj = Odometer(n, odom_frame_id, body_frame_id, wheel_base, wheel_radius);
    SLAM slam_obj = SLAM(n, odom_frame_id, wheel_base, wheel_radius, odo_obj, vx_std, the_std);
    ros::spin();



    return 0;

}

