/// \file  turtle_interface.cpp
/// \brief low-level control and sensor routines for turtlebot: boba
///
/// PARAMETERS:
///     left_wheel_joint (string):  The name of the left wheel joint
///     right_wheel_joint (string):  The name of the right wheel joint
///     wheel_base (double):  The distance between the wheels
///     wheel_radius (double):  The radius of the wheels
/// PUBLISHES:
///     wheel_pub (nuturtlebot::WheelCommands): Publishes the wheel command to turtlebot
///     joint_pub (sensor_msgs::JointState): Publishes the joint state to the odometry
/// SUBSCRIBES:
///     vel_sub (geometry_msgs::Twist): velocity command from user (teleop)
///     sensor_sub (sensor_msgs::JointState): sensor information from the turtlebot (wheel encoders, etc.)



#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"


/// \brief interface with low-level control and sensor
class TurtleInterface
{
    private:
        ros::NodeHandle nh;
        ros::Subscriber vel_sub;
        ros::Subscriber sensor_sub;
        ros::Publisher wheel_pub;
        ros::Publisher joint_pub;
        ros::Timer timer;


        rigid2d::DiffDrive dd;

        double curr_lin_vel;
        double curr_ang_vel;

        double max_trans_vel;
        double max_rota_vel;
        double max_wheel_vel;

        int curr_left_encoder;
        int curr_right_encoder;
        int pre_left_encoder;
        int pre_right_encoder;
        bool first_encoder_flag;
        int first_left_encoder;
        int first_right_encoder;

        bool wheel_pub_flag;
        bool joint_pub_flag;

        std::string left_wheel_joint;
        std::string right_wheel_joint;


        int counter;


    public:

        /// \brief create the initial setup for interface
        ///
        /// \param nh - the node handle for ROS
        /// \param left_wheel_joint_str - The name of the left wheel joint
        /// \param right_wheel_joint_str - The name of the right wheel joint
        /// \param wheel_base_val - The distance between the wheels 
        /// \param wheel_radius_val - The radius of the wheels
        TurtleInterface(ros::NodeHandle nh, std::string left_wheel_joint_str, std::string right_wheel_joint_str, double wheel_base_val, double wheel_radius_val):
        left_wheel_joint(left_wheel_joint_str),
        right_wheel_joint(right_wheel_joint_str),
        timer(nh.createTimer(ros::Duration(0.01), &TurtleInterface::main_loop, this)),
        vel_sub(nh.subscribe("cmd_vel", 1000, &TurtleInterface::callback_vel, this)),
        sensor_sub(nh.subscribe("sensor_data", 1000, &TurtleInterface::callback_sensor, this)),
        wheel_pub(nh.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 100)),
        wheel_pub_flag(false),
        joint_pub_flag(false),
        joint_pub(nh.advertise<sensor_msgs::JointState>("joint_states", 100)),
        dd(rigid2d::DiffDrive(wheel_base_val, wheel_radius_val)),
        curr_lin_vel(0.0),
        curr_ang_vel(0.0),
        max_trans_vel(0.22),
        max_rota_vel(2.84),
        curr_left_encoder(0),
        curr_right_encoder(0),
        pre_left_encoder(0),
        pre_right_encoder(0),
        first_encoder_flag(true),
        first_left_encoder(0),
        first_right_encoder(0),
        counter(0)
        {

            rigid2d::Vector2D max_lin = {0.0,0.0};
            rigid2d::Twist2D max_ts = rigid2d::Twist2D(max_rota_vel, max_lin);
            rigid2d::Vector2D wheel_vel = dd.calculateWheelVelocity(max_ts);
            max_wheel_vel = (fabs(wheel_vel.x)/100.0)*0.99;
        }

        /// \brief publish new wheel command
        void publishWheelCommand()
        {
            nuturtlebot::WheelCommands wheel_cmd;

            rigid2d::Vector2D lin = {curr_lin_vel,0.0};
            rigid2d::Twist2D ts = rigid2d::Twist2D(curr_ang_vel, lin);
            rigid2d::Vector2D wheel_vel = dd.calculateWheelVelocity(ts);

            double delta_wheel_left = wheel_vel.x;
            double delta_wheel_right = wheel_vel.y;

            wheel_cmd.left_velocity = ((delta_wheel_left/100.0)/max_wheel_vel)*256;
            wheel_cmd.right_velocity = ((delta_wheel_right/100.0)/max_wheel_vel)*256;

            wheel_pub.publish(wheel_cmd);
        }

        /// \brief publish new joint state
        void publishJointState()
        {
            
            int encoder_max_tick = 4096;

            double curr_rad_left = ((double)curr_left_encoder/(double)encoder_max_tick)*2*rigid2d::PI;
            double curr_rad_right = ((double)curr_right_encoder/(double)encoder_max_tick)*2*rigid2d::PI;

            double delta_rad_left = ((double)(curr_left_encoder - pre_left_encoder)/(double)encoder_max_tick)*2*rigid2d::PI;//*4;
            double delta_rad_right = ((double)(curr_right_encoder - pre_right_encoder)/(double)encoder_max_tick)*2*rigid2d::PI;//*4;

            sensor_msgs::JointState joint_msg;
            joint_msg.header.stamp = ros::Time::now();
            joint_msg.name.push_back(left_wheel_joint);
            joint_msg.name.push_back(right_wheel_joint);
            joint_msg.position.push_back(curr_rad_left);
            joint_msg.position.push_back(curr_rad_right);
            joint_msg.velocity.push_back(delta_rad_left);
            joint_msg.velocity.push_back(delta_rad_right);

            joint_pub.publish(joint_msg);

        }

        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &)
        {


            if (wheel_pub_flag)
            {
                publishWheelCommand();
            }
            
            if (joint_pub_flag)
            {

                publishJointState();
            }



            
        }

        
        /// \brief callback function for velocity command
        /// \param vel - velocity command
        void callback_vel(const geometry_msgs::Twist & vel)
        {
            if (vel.linear.x > max_trans_vel)
            {
                curr_lin_vel = max_trans_vel;
            }
            else if (vel.linear.x < -max_trans_vel)
            {
                curr_lin_vel = -max_trans_vel;
            }
            else
            {
                curr_lin_vel = vel.linear.x;
            }

            if (vel.angular.z > max_rota_vel)
            {
                curr_ang_vel = max_rota_vel;
            }
            else if (vel.angular.z < -max_rota_vel)
            {
                curr_ang_vel = -max_rota_vel;
            }else
            {
                curr_ang_vel = vel.angular.z;
            }
            wheel_pub_flag = true;
        }

        /// \brief callback function for sensor data
        /// \param sensor_data - incoming sensor data
        void callback_sensor(const nuturtlebot::SensorData & sensor_data)
        {
            if (first_encoder_flag){
                first_left_encoder = sensor_data.left_encoder;
                first_right_encoder = sensor_data.right_encoder;
                first_encoder_flag = false;
            }
            if (counter < 5){
                counter++;
            }else{
                pre_left_encoder = curr_left_encoder;
                pre_right_encoder = curr_right_encoder;
                counter = 0;
            }

            curr_left_encoder = sensor_data.left_encoder - first_left_encoder;
            curr_right_encoder = sensor_data.right_encoder - first_right_encoder;
            joint_pub_flag = true;
            
        }


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_interface");
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


    TurtleInterface interface = TurtleInterface(n, left_wheel_joint, right_wheel_joint, wheel_base, wheel_radius);
    ros::spin();


    return 0;

}


