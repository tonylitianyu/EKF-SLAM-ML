#include "../include/rigid2d/diff_drive.hpp"
#include<iostream>



rigid2d::DiffDrive::DiffDrive(double wheel_base, double wheel_radius){
    wheel_b = wheel_base;
    wheel_r = wheel_radius;

    position = {0.0, 0.0};
    theta = 0.0;

}


rigid2d::DiffDrive::DiffDrive(double wheel_base, double wheel_radius, Vector2D& init_pos, double init_theta){
    wheel_b = wheel_base;
    wheel_r = wheel_radius;

    position = {init_pos.x, init_pos.y};
    theta = init_theta;
}


rigid2d::Vector2D rigid2d::DiffDrive::calculateWheelVelocity(const Twist2D& ts){


    double body_ang = ts.angular();
    double body_vel_x = ts.linearX();
    double D = wheel_b*0.5;
    double r = wheel_r;

    double wheel_vel_x_L = -(D/r)*body_ang + (1/r)*body_vel_x;  //Using equation (1), body twist to left wheel velocity
    double wheel_vel_x_R = (D/r)*body_ang + (1/r)*body_vel_x;   //Using equation (2), body twist to right wheel velocity

    return {wheel_vel_x_L, wheel_vel_x_R};
}

rigid2d::Twist2D rigid2d::DiffDrive::getBodyTwistForUpdate(double left_angle, double right_angle){
    double D = wheel_b*0.5;
    double r = wheel_r;
    double ts_angle = (r/(2*D))*(right_angle-left_angle);   //Using equation (3), find the twist angle from wheel angle
    double ts_x = (r/2)*(right_angle+left_angle);           //Using equation (4), find the twist x velocity from wheel angle

    rigid2d::Vector2D lin = {ts_x, 0.0};
    rigid2d::Twist2D body_twist = Twist2D(ts_angle, lin);
    return body_twist;
}


void rigid2d::DiffDrive::updatePose(double left_angle, double right_angle){
    rigid2d::Twist2D body_twist = rigid2d::DiffDrive::getBodyTwistForUpdate(left_angle, right_angle);

    rigid2d::Transform2D t_bbq = integrateTwist(body_twist);

    rigid2d::Vector2D q_tran = {t_bbq.x(), t_bbq.y()};
    rigid2d::Twist2D q_b = Twist2D( t_bbq.theta(), q_tran);


    rigid2d::Transform2D t_wb = Transform2D(theta);
    rigid2d::Twist2D q = t_wb.adjConvert(q_b);

    position.x += q.linearX();
    position.y += q.linearY();
    theta += q.angular();

}


rigid2d::Vector2D rigid2d::DiffDrive::getPosition(){
    return position;
}

double rigid2d::DiffDrive::getTheta(){
    return theta;
}