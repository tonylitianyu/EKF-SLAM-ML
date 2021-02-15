#include "ros/ros.h"
#include <catch_ros/catch.hpp>
#include "nuturtlebot/WheelCommands.h"
#include "geometry_msgs/Twist.h"
#include "nuturtlebot/SensorData.h"
#include "sensor_msgs/JointState.h"


void callback_trans_wheel(const nuturtlebot::WheelCommands & cmd)
{
    REQUIRE( cmd.left_velocity == 243 );
    REQUIRE( cmd.right_velocity == 243 );
}

TEST_CASE("Pure Translation", "[cmd_vel]")
{
    ros::NodeHandle nh;
    const auto wheel_sub = nh.subscribe("wheel_cmd", 1000, callback_trans_wheel);

	const auto cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.22;
    cmd_pub.publish(vel_msg);
    ros::Rate r(100.0);

    for (int i = 0; ros::ok() && i != 100; ++i){
        ros::spinOnce();
        r.sleep();
    }


}

void callback_rota_wheel(const nuturtlebot::WheelCommands & cmd)
{
    REQUIRE( cmd.left_velocity == -250 );
    REQUIRE( cmd.right_velocity == 250 );
}

TEST_CASE("Pure Rotation", "[cmd_vel]")
{
    ros::NodeHandle nh;
    const auto wheel_sub = nh.subscribe("wheel_cmd", 1000, callback_rota_wheel);

	const auto cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 2.82;
    cmd_pub.publish(vel_msg);
    ros::Rate r(100.0);

    for (int i = 0; ros::ok() && i != 100; ++i){
        ros::spinOnce();
        r.sleep();
    }


}



void callback_joint_states(const sensor_msgs::JointState &joints)
{
    
    REQUIRE( joints.position[0] == Approx(1.53398) );
    REQUIRE( joints.position[1] == Approx(1.53398) );

}

TEST_CASE("Encoder Data", "[sensor_data]")
{
    ros::NodeHandle nh;

	const auto init_sensor_pub = nh.advertise<nuturtlebot::SensorData>("sensor_data", 100, true);

    nuturtlebot::SensorData init_sensor;
    init_sensor.left_encoder = 0;
    init_sensor.right_encoder = 0;

    init_sensor_pub.publish(init_sensor);

	const auto joint_sub = nh.subscribe("joint_states", 1000, callback_joint_states);
    const auto sensor_pub = nh.advertise<nuturtlebot::SensorData>("sensor_data", 100, true);

    nuturtlebot::SensorData sensor_msg;
    sensor_msg.left_encoder = 1000;
    sensor_msg.right_encoder = 1000;

    sensor_pub.publish(sensor_msg);

    
    ros::Rate r(100.0);

    for (int i = 0; ros::ok() && i != 200; ++i){
        
        ros::spinOnce();
        r.sleep();
    }


}