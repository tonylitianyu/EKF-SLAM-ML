#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for kinematics of wheeled mobile robots.

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include "rigid2d.hpp"



namespace rigid2d
{
    /// \brief differential drive model calculation
    class DiffDrive{
        public:
            DiffDrive(double wheel_base, double wheel_radius);

            DiffDrive(double wheel_base, double wheel_radius, Vector2D& init_pos, double init_theta);

            Vector2D calculateWheelVelocity(const Twist2D& ts);

            Twist2D getBodyTwistForUpdate(double left_angle, double right_angle);


            void updatePose(double left_angle, double right_angle);

            Vector2D getPosition();
            double getTheta();


        private:
            double wheel_b;
            double wheel_r;
            Vector2D position;
            double theta;
    };




}


#endif