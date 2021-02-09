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

            /// \brief create a diffdrive object with zero configurations
            /// \param wheel_base - the distance between two wheels
            /// \param wheel_radius - the radius of the wheels
            DiffDrive(double wheel_base, double wheel_radius);

            /// \brief create a diffdrive object with given parameters and configurations
            /// \param wheel_base - the distance between two wheels
            /// \param wheel_radius - the radius of the wheels
            /// \param init_pos - the initial linear configurations
            /// \param init_theta - the initial angular configuration
            DiffDrive(double wheel_base, double wheel_radius, Vector2D& init_pos, double init_theta);


            /// \brief calculate wheel velocity
            /// \param ts - the distance between two wheels
            /// \return wheel velocity (Left, Right)
            Vector2D calculateWheelVelocity(const Twist2D& ts);


            /// \brief calculate body twist using update wheel angle
            /// \param left_angle - the left angle change in one time unit
            /// \param right_angle - the right angle change in one time unit
            /// \return new body twist
            Twist2D getBodyTwistForUpdate(double left_angle, double right_angle);

            /// \brief update the configuration of the diff drive robot
            /// \param left_angle - the left angle change in one time unit
            /// \param right_angle - the right angle change in one time unit
            void updatePose(double left_angle, double right_angle);

            /// \brief get the x and y position of the robot
            /// \return the x and y position of the robot
            Vector2D getPosition();

            /// \brief get the angular orientation of the robot
            /// \return the agualr orientation of the robot
            double getTheta();


        private:
            double wheel_b;
            double wheel_r;
            Vector2D position;
            double theta;
    };




}


#endif