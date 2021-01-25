
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "std_srvs/Empty.h"
#include "trect/Start.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include <cstdio>
#include <cmath>


enum class State {PAUSE, START, LOW_RIGHT,TOP_RIGHT,TOP_LEFT, LOW_LEFT};
static State state = State::PAUSE;

class TurtleRect
{
    private:
        ros::Publisher vel_pub;
        ros::Subscriber pose_sub;
        ros::ServiceServer start_srv;
        ros::ServiceClient clear_client;
        ros::ServiceClient teleport_client;
        ros::ServiceClient pen_client;
        ros::Timer timer;

        bool start_following;
        double max_xdot;
        double max_wdot;

        double init_x;
        double init_y;
        double curr_x;
        double curr_y;
        double curr_theta;



        double rect_x_arr[4];
        double rect_y_arr[4];
        double theta_arr[4];





    public:
        TurtleRect(ros::NodeHandle*nh, double max_xdot_vel, double max_wdot_vel, int pub_freq)
        {
            vel_pub = nh->advertise<geometry_msgs::Twist>("turtle1/cmd_vel", pub_freq);

            pose_sub = nh->subscribe("turtle1/pose", 1000, &TurtleRect::callback_pose, this);

            start_srv = nh->advertiseService("start", &TurtleRect::callback_start_service, this);

            clear_client = nh->serviceClient<std_srvs::Empty>("clear");

            teleport_client = nh->serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");

            pen_client = nh->serviceClient<turtlesim::SetPen>("turtle1/set_pen");


            timer = nh->createTimer(ros::Duration(0.1), &TurtleRect::main_loop, this);

            max_xdot = max_xdot_vel;
            max_wdot = max_wdot_vel;



        }
        
        void start_setup()
        {
            std_srvs::Empty clear_srv;
            clear_client.call(clear_srv);

            turtlesim::SetPen pen_srv;
            pen_srv.request.width = 3;
            pen_srv.request.off = 1;
            pen_client.call(pen_srv);


            turtlesim::TeleportAbsolute move_to_initial_srv;
            move_to_initial_srv.request.x = init_x;
            move_to_initial_srv.request.y = init_y;
            move_to_initial_srv.request.theta = 0.0;
            teleport_client.call(move_to_initial_srv);


            pen_srv.request.r = 255;
            pen_srv.request.g = 255;
            pen_srv.request.b = 255;
            pen_srv.request.width = 3;
            pen_srv.request.off = 0;
            pen_client.call(pen_srv);

            for (int i = 0; i < 4; i++)
            {
                turtlesim::TeleportAbsolute teleport_srv;
                teleport_srv.request.x = rect_x_arr[i];
                teleport_srv.request.y = rect_y_arr[i];
                teleport_srv.request.theta = 0.0;

                teleport_client.call(teleport_srv);
            }


            pen_srv.request.r = 0;
            pen_srv.request.g = 0;
            pen_srv.request.b = 0;
            pen_srv.request.width = 3;
            pen_srv.request.off = 0;
            pen_client.call(pen_srv);


        }

        void get_next_state(){
            if (state == State::LOW_LEFT){
                state = State::LOW_RIGHT;
            }else{
                state = static_cast<State>(static_cast<int>(state) + 1);
            }
        }


        void move_to_next_goal(double x, double y, double theta){
            geometry_msgs::Twist msg;

            //rotate
            
            if (abs(curr_theta-theta) > 0.01){
                double ang_vel = abs(curr_theta-theta);
                if (ang_vel > max_wdot){
                    ang_vel = max_wdot;
                }
                if (ang_vel < 0.1){
                    ang_vel = 0.1;
                }

                msg.angular.z = ang_vel;
            }else{
                //move forward
                double lin_dis = sqrt(pow(x-curr_x,2)+pow(y-curr_y,2));

                if (lin_dis > 0.04){
                    double lin_vel = lin_dis;
                    if (lin_vel > max_xdot){
                        lin_vel = max_xdot;
                    }
                    if (lin_vel < 0.2){
                        lin_vel = 0.2;
                    }
                    msg.linear.x = lin_dis;
                }else{
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.0;
                    get_next_state();
                }
            }

            vel_pub.publish(msg);

        }


        void main_loop(const ros::TimerEvent &)
        {

            switch (state)
            {
            case State::PAUSE:
                break;
            case State::START:
                start_setup();
                state = State::LOW_RIGHT;
                break;
            case State::LOW_RIGHT:
                move_to_next_goal(rect_x_arr[0], rect_y_arr[0], theta_arr[0]);
                break;
            case State::TOP_RIGHT:
                move_to_next_goal(rect_x_arr[1], rect_y_arr[1], theta_arr[1]);
                break;
            case State::TOP_LEFT:
                move_to_next_goal(rect_x_arr[2], rect_y_arr[2], theta_arr[2]);
                break;
            case State::LOW_LEFT:
                move_to_next_goal(rect_x_arr[3], rect_y_arr[3], theta_arr[3]);
                break;
            
            default:
                throw std::logic_error("Invalid State");
                break;
            }
        }

        void callback_pose(const turtlesim::Pose &pose)
        {
            curr_x = pose.x;
            curr_y = pose.y;
            curr_theta = pose.theta;
            

        }

        bool callback_start_service(trect::Start::Request &req, trect::Start::Response &res)
        {
            init_x = req.init_x;
            init_y = req.init_y;
            double width = req.width;
            double height = req.height;

            double rect_x[4] = {init_x+width, init_x+width, init_x, init_x};
            double rect_y[4] = {init_y, init_y+height, init_y+height, init_y};
            double thea[4] = {0.0,1.57,3.14,-1.57};

            for (int k = 0; k < 4; k++){
                rect_x_arr[k] = rect_x[k];
                rect_y_arr[k] = rect_y[k];
                theta_arr[k] = thea[k];
            }

            state = State::START;
        

            return true;
        }

};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle n;
    
    double max_xdot_param;
    double max_wdot_param;
    int frequency_param;

    if (n.getParam("max_xdot", max_xdot_param))
    {
        ROS_INFO("max_xdot: %f", max_xdot_param);
    }else
    {
        ROS_ERROR("Unable to get param 'max_xdot'");
    }

    if (n.getParam("max_wdot", max_wdot_param))
    {
        ROS_INFO("max_wdot: %f", max_wdot_param);
    }else
    {
        ROS_ERROR("Unable to get param 'max_wdot'");
    }

    if (n.getParam("frequency", frequency_param))
    {
        ROS_INFO("frequency: %d", frequency_param);
    }else
    {
        ROS_ERROR("Unable to get param 'frequency'");
    }


    TurtleRect tr = TurtleRect(&n, max_xdot_param, max_wdot_param,frequency_param);
    ros::spin();



    return 0;

}

