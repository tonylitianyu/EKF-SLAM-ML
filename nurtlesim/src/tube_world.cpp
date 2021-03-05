/// \file  tube_world.cpp
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
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "rigid2d/diff_drive.hpp"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/Path.h"
#include <random>
#include <vector>
#include <cmath>

class TubeWorld{
    private:
        ros::Publisher true_tube_pub;
        ros::Timer timer;
        ros::Subscriber vel_sub;
        ros::Publisher joint_pub;
        ros::Publisher robot_truth_marker_pub;
        ros::Publisher path_pub;
        ros::Publisher fake_tube_pub;

        int fake_tube_counter;

        nav_msgs::Path real_path;

        std::vector<double> coor_x;
        std::vector<double> coor_y;
        double radius;
        double covar_sensor_x;
        double covar_sensor_y;
        double max_visible_dis;

        double x_vel;
        double ang_vel;

        rigid2d::DiffDrive dd;
        double wheel_base;
        double wheel_radius;
        double left_wheel_angle;
        double right_wheel_angle;

        std::string left_wheel_joint;
        std::string right_wheel_joint;

        double slip_min;
        double slip_max;

        double vx_std;
        double the_std;


        unsigned seed;

        
    public:
        TubeWorld(ros::NodeHandle nh, std::vector<double> coor_x, std::vector<double> coor_y, double radius, double covar_sensor_x,
                double covar_sensor_y, double max_visible_dis, std::string left_wheel_joint_str, std::string right_wheel_joint_str, double wheel_base_val, double wheel_radius_val,
                double vx_std, double the_std, double slip_min, double slip_max):
        timer(nh.createTimer(ros::Duration(0.01), &TubeWorld::main_loop, this)),
        coor_x(coor_x),
        coor_y(coor_y),
        radius(radius),
        covar_sensor_x(covar_sensor_x),
        covar_sensor_y(covar_sensor_y),
        max_visible_dis(max_visible_dis),
        true_tube_pub(nh.advertise<visualization_msgs::MarkerArray>("true_sensor", 10, true)),
        fake_tube_pub(nh.advertise<visualization_msgs::MarkerArray>("fake_sensor", 10, true)),
        path_pub(nh.advertise<nav_msgs::Path>("real_path", 100)),
        robot_truth_marker_pub(nh.advertise<visualization_msgs::Marker>("ground_truth_turtle", 100)),
        x_vel(0.0),
        ang_vel(0.0),
        wheel_base(wheel_base_val),
        wheel_radius(wheel_radius_val),
        dd(rigid2d::DiffDrive(wheel_base_val, wheel_radius_val)),
        vel_sub(nh.subscribe("cmd_vel", 1000, &TubeWorld::callback_vel, this)),
        joint_pub(nh.advertise<sensor_msgs::JointState>("joint_states", 100)),
        left_wheel_angle(0.0),
        right_wheel_angle(0.0),
        left_wheel_joint(left_wheel_joint_str),
        right_wheel_joint(right_wheel_joint_str),
        fake_tube_counter(0),
        vx_std(vx_std),
        the_std(the_std),
        slip_min(slip_min),
        slip_max(slip_max)

        {
            visualization_msgs::MarkerArray true_tube_marker_array;
            for (int i = 0; i < coor_x.size(); i++){
                visualization_msgs::Marker true_tube_marker;
                true_tube_marker.header.frame_id = "world";
                true_tube_marker.header.stamp = ros::Time::now();
                true_tube_marker.ns = "real";
                true_tube_marker.action = visualization_msgs::Marker::ADD;

                true_tube_marker.id = i;
                true_tube_marker.type = visualization_msgs::Marker::CYLINDER;
                true_tube_marker.scale.x = 0.1;
                true_tube_marker.scale.y = 0.1;
                true_tube_marker.scale.z = 0.3;
                true_tube_marker.color.g = 1.0;
                true_tube_marker.color.a = 1.0;

                true_tube_marker.pose.position.x = coor_x[i];
                true_tube_marker.pose.position.y = coor_y[i];
                true_tube_marker.pose.position.z = 0.15;
                true_tube_marker.pose.orientation.w = 1.0;

                true_tube_marker_array.markers.push_back(true_tube_marker);

            }

            true_tube_pub.publish(true_tube_marker_array);
        }

        std::mt19937 & get_random(){
            static std::random_device rd{}; 
            static std::mt19937 mt{rd()};

            return mt;
        }


        void callback_vel(const geometry_msgs::Twist &vel)
        {
            if (vel.linear.x < 0.0001 && vel.linear.x > -0.0001){
                x_vel = vel.linear.x;
            }else{
                std::normal_distribution<> x_noise(0.0, vx_std);
                x_vel = vel.linear.x + x_noise(get_random());
            }

            
            if (vel.angular.z < 0.0001 && vel.angular.z > -0.0001){
                ang_vel = vel.angular.z;
            }else{
                std::normal_distribution<> the_noise(0.0, the_std);
                ang_vel = vel.angular.z + the_noise(get_random());
            }



        }

        void publishJointState(){
            rigid2d::Vector2D lin = {x_vel,0.0};
            
            rigid2d::Twist2D ts = rigid2d::Twist2D(ang_vel, lin);
            rigid2d::Vector2D wheel_vel = dd.calculateWheelVelocity(ts);
            

            double delta_wheel_left = (wheel_vel.x/100.0);   //100.0 is the timer frequency in odometer
            double delta_wheel_right = (wheel_vel.y/100.0);

            std::uniform_real_distribution<> left_noise(slip_min, slip_max);
            std::uniform_real_distribution<> right_noise(slip_min, slip_max);
            delta_wheel_left = delta_wheel_left*left_noise(get_random());
            delta_wheel_right = delta_wheel_right*right_noise(get_random());
            


            dd.updatePose(delta_wheel_left, delta_wheel_right);

            checkCollision();

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

        void publishTurtleFrame(){
            tf2::Quaternion q;
            q.setRPY(0,0,dd.getTheta());
            static tf2_ros::TransformBroadcaster odom_br;
            geometry_msgs::TransformStamped odom_transform;
            odom_transform.header.stamp = ros::Time::now();
            odom_transform.header.frame_id = "world";
            odom_transform.child_frame_id = "turtle";
            odom_transform.transform.translation.x = dd.getPosition().x;
            odom_transform.transform.translation.y = dd.getPosition().y;
            odom_transform.transform.translation.z = 0.0;
            odom_transform.transform.rotation.x = q.x();
            odom_transform.transform.rotation.y = q.y();
            odom_transform.transform.rotation.z = q.z();
            odom_transform.transform.rotation.w = q.w();
            odom_br.sendTransform(odom_transform);

            visualization_msgs::Marker true_turtle_marker;
            true_turtle_marker.header.frame_id = "world";
            true_turtle_marker.header.stamp = ros::Time::now();
            true_turtle_marker.ns = "real_turtle";
            true_turtle_marker.action = visualization_msgs::Marker::ADD;

            true_turtle_marker.id = 0;
            true_turtle_marker.type = visualization_msgs::Marker::CUBE;
            true_turtle_marker.scale.x = 0.1;
            true_turtle_marker.scale.y = 0.1;
            true_turtle_marker.scale.z = 0.1;
            true_turtle_marker.color.g = 1.0;
            true_turtle_marker.color.a = 1.0;

            true_turtle_marker.pose.position.x = dd.getPosition().x;
            true_turtle_marker.pose.position.y = dd.getPosition().y;
            true_turtle_marker.pose.position.z = 0.05;

            true_turtle_marker.pose.orientation.x = q.x();
            true_turtle_marker.pose.orientation.y = q.y();
            true_turtle_marker.pose.orientation.z = q.z();
            true_turtle_marker.pose.orientation.w = q.w();
            robot_truth_marker_pub.publish(true_turtle_marker);

            //add path
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

            path_pub.publish(real_path);
        }


        double distanceToTube(int m){
            return sqrt(pow(dd.getPosition().x-coor_x[m],2)+pow(dd.getPosition().y-coor_y[m],2));
        }

        std::vector<double> findCollisionTubePos(){
            std::vector<double> tube_pos;
            for (int i = 0; i < coor_x.size(); i++){
                double dis = distanceToTube(i);
                if (dis < (radius + (wheel_base/2))){
                    tube_pos.push_back(coor_x[i]);
                    tube_pos.push_back(coor_y[i]);
                    return tube_pos;
                }
            }

            return tube_pos;
        }

        void checkCollision(){
            std::vector<double> colliding_tube = findCollisionTubePos();
            if (colliding_tube.size() != 0){
                moveAfterCollision(colliding_tube[0], colliding_tube[1]);
            }
        }

        void moveAfterCollision(double tube_x, double tube_y){


            double dx = tube_x-dd.getPosition().x;
            double dy = tube_y-dd.getPosition().y;

            double angle = atan2(dy, dx);
            double correct_dis = radius + (wheel_base/2);
            double correct_dy = correct_dis*sin(angle);
            double correct_dx = correct_dis*cos(angle);

            double correct_x = tube_x - correct_dx;
            double correct_y = tube_y - correct_dy;


            rigid2d::Vector2D set_tran = rigid2d::Vector2D(correct_x,correct_y);
            dd = rigid2d::DiffDrive(wheel_base, wheel_radius, set_tran, dd.getTheta());

        }


        void publishFakeSensor(){
            visualization_msgs::MarkerArray fake_tube_marker_array;
            std::normal_distribution<> x_noise(0.0, covar_sensor_x);
            std::normal_distribution<> y_noise(0.0, covar_sensor_y);
            for (int j = 0; j < coor_x.size(); j++){
                visualization_msgs::Marker fake_tube_marker;
                fake_tube_marker.header.frame_id = "turtle";
                fake_tube_marker.header.stamp = ros::Time::now();
                fake_tube_marker.ns = "fake";
                fake_tube_marker.lifetime = ros::Duration(0.1);
                

                fake_tube_marker.id = j;
                fake_tube_marker.type = visualization_msgs::Marker::CYLINDER;
                fake_tube_marker.scale.x = 0.1;
                fake_tube_marker.scale.y = 0.1;
                fake_tube_marker.scale.z = 0.3;
                fake_tube_marker.color.r = 1.0;
                fake_tube_marker.color.a = 1.0;


                rigid2d::Vector2D trans = {dd.getPosition().x, dd.getPosition().y};
                rigid2d::Transform2D Ttw = rigid2d::Transform2D(trans, dd.getTheta()).inv();

                rigid2d::Vector2D world_tube = {coor_x[j], coor_y[j]};
                rigid2d::Vector2D turtle_tube = Ttw(world_tube);

                
                fake_tube_marker.pose.position.x = turtle_tube.x+x_noise(get_random());
                fake_tube_marker.pose.position.y = turtle_tube.y+y_noise(get_random());
                fake_tube_marker.pose.position.z = 0.15;
                fake_tube_marker.pose.orientation.w = 1.0;


                if (distanceToTube(j) > max_visible_dis){
                    fake_tube_marker.action = visualization_msgs::Marker::DELETE;
                }else{
                    fake_tube_marker.action = visualization_msgs::Marker::ADD;
                }


                fake_tube_marker_array.markers.push_back(fake_tube_marker);
            }

            fake_tube_pub.publish(fake_tube_marker_array);
        }




        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            publishTurtleFrame();
            publishJointState();

            if (fake_tube_counter == 10){
                publishFakeSensor();
                fake_tube_counter = 0;
            }else{
                fake_tube_counter++;
            }
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

    double vx_mu;
    double vx_std;
    double the_mu;
    double the_std;
    double slip_min;
    double slip_max;
    double covar_sensor_x;
    double covar_sensor_y;
    double max_visible_dis;

    std::vector<double> tube_coor_x;
    std::vector<double> tube_coor_y;
    double tube_radius;



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

    if (n.getParam("vx_mu", vx_mu))
    {
        ROS_INFO("vx_mu: %f", vx_mu);
    }else
    {
        ROS_ERROR("Unable to get param 'vx_mu'");
    }

    if (n.getParam("vx_std", vx_std))
    {
        ROS_INFO("vx_std: %f", vx_std);
    }else
    {
        ROS_ERROR("Unable to get param 'vx_std'");
    }

    if (n.getParam("the_mu", the_mu))
    {
        ROS_INFO("the_mu: %f", the_mu);
    }else
    {
        ROS_ERROR("Unable to get param 'the_mu'");
    }

    if (n.getParam("the_std", the_std))
    {
        ROS_INFO("the_std: %f", the_std);
    }else
    {
        ROS_ERROR("Unable to get param 'the_std'");
    }
    if (n.getParam("slip_min", slip_min))
    {
        ROS_INFO("slip_min: %f", slip_min);
    }else
    {
        ROS_ERROR("Unable to get param 'slip_min'");
    }

    if (n.getParam("slip_max", slip_max))
    {
        ROS_INFO("slip_max: %f", slip_max);
    }else
    {
        ROS_ERROR("Unable to get param 'slip_max'");
    }

    if (n.getParam("tube_coor_x", tube_coor_x))
    {
        ROS_INFO("tube_coor_x");
    }else
    {
        ROS_ERROR("Unable to get param 'tube_coor_x'");
    }

    if (n.getParam("tube_coor_y", tube_coor_y))
    {
        ROS_INFO("tube_coor_y");
    }else
    {
        ROS_ERROR("Unable to get param 'tube_coor_y'");
    }

    if (n.getParam("tube_radius", tube_radius))
    {
        ROS_INFO("tube_radius: %f", tube_radius);
    }else
    {
        ROS_ERROR("Unable to get param 'tube_radius'");
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

    TubeWorld tw = TubeWorld(n, tube_coor_x, tube_coor_y, tube_radius, covar_sensor_x, covar_sensor_y, max_visible_dis, left_wheel_joint, right_wheel_joint, wheel_base, wheel_radius, 
                                vx_std, the_std, slip_min, slip_max);



    ros::spin();



    return 0;

}