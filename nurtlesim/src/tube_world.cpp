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
#include "sensor_msgs/JointState.h"
#include "rigid2d/diff_drive.hpp"
#include <random>


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

        //noise
        double vx_mu;
        double vx_std;
        double the_mu;
        double the_std;
        double slip_min;
        double slip_max;

        unsigned seed;

    public:
        /// \brief create the initial setup for the simulator
        ///
        /// \param nh - the node handle for ROS
        /// \param left_wheel_joint_str - The name of the left wheel joint
        /// \param right_wheel_joint_str - The name of the right wheel joint
        /// \param wheel_base_val - The distance between the wheels 
        /// \param wheel_radius_val - The radius of the wheels
        /// \param vx_mu - Gaussian noise mean for commanded twist tranlational speed
        /// \param vx_std - Gaussian noise std for commanded twist tranlational speed
        /// \param the_mu - Gaussian noise mean for commanded twist rotational speed
        /// \param the_std - Gaussian noise std for commanded twist rotational speed
        /// \param slip_min - Minimum slip noise
        /// \param slip_max - Maximum slip noise
        FakeTurtle(ros::NodeHandle nh, std::string left_wheel_joint_str, std::string right_wheel_joint_str, 
                            double wheel_base_val, double wheel_radius_val, double vx_mu, double vx_std, 
                            double the_mu, double the_std, double slip_min, double slip_max):
        timer(nh.createTimer(ros::Duration(0.01), &FakeTurtle::main_loop, this)),
        vel_sub(nh.subscribe("cmd_vel", 1000, &FakeTurtle::callback_vel, this)),
        joint_pub(nh.advertise<sensor_msgs::JointState>("joint_states", 100)),
        dd(rigid2d::DiffDrive(wheel_base_val, wheel_radius_val)),
        left_wheel_angle(0.0),
        right_wheel_angle(0.0),
        left_wheel_joint(left_wheel_joint_str),
        right_wheel_joint(right_wheel_joint_str),
        vx_mu(vx_mu),
        vx_std(vx_std),
        the_mu(the_mu),
        the_std(the_std),
        slip_min(slip_min),
        slip_max(slip_max),
        //source:https://stackoverflow.com/questions/32889309/adding-gaussian-noise
        seed(std::chrono::system_clock::now().time_since_epoch().count())
        {
        }

        /// \brief callback function for velocity command
        /// \param vel - velocity command
        void callback_vel(const geometry_msgs::Twist &vel)
        {
            if (vel.linear.x < 0.0001 || vel.linear.x > -0.0001){
                x_vel = vel.linear.x;
            }else{
                std::default_random_engine random_x(seed);
                std::normal_distribution<double> x_noise(vx_mu, vx_std);
                x_vel = vel.linear.x + x_noise(random_x);
            }

            
            if (vel.angular.z < 0.0001 || vel.angular.z > -0.0001){
                ang_vel = vel.angular.z;
            }else{
                std::default_random_engine random_the(seed);
                std::normal_distribution<double> the_noise(the_mu, the_std);
                ang_vel = vel.angular.z + the_noise(random_the);
            }


        }

        void publishJointState(){
            rigid2d::Vector2D lin = {x_vel,y_vel};
            rigid2d::Twist2D ts = rigid2d::Twist2D(ang_vel, lin);
            rigid2d::Vector2D wheel_vel = dd.calculateWheelVelocity(ts);



            double delta_wheel_left = (wheel_vel.x/100.0);   //100.0 is the timer frequency in odometer
            double delta_wheel_right = (wheel_vel.y/100.0);

            std::default_random_engine random_left(seed);
            std::default_random_engine random_right(seed);
            std::uniform_real_distribution<double> left_noise(slip_min, slip_max);
            std::uniform_real_distribution<double> right_noise(slip_min, slip_max);

            delta_wheel_left += left_noise(random_left)*delta_wheel_left;
            delta_wheel_right += right_noise(random_right)*delta_wheel_right;

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

    double vx_mu;
    double vx_std;
    double the_mu;
    double the_std;
    double slip_min;
    double slip_max;

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


    FakeTurtle ft = FakeTurtle(n, left_wheel_joint, right_wheel_joint, wheel_base, wheel_radius,
                                vx_mu, vx_std, the_mu, the_std, slip_min, slip_max);
    ros::spin();



    return 0;

}