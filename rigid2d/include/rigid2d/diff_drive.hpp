#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for kinematics of wheeled mobile robots.

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>



namespace rigid2d
{
    /// \brief differential drive model calculation
    class DiffDrive{
        public:
            DiffDrive();


        private:
            double wheel_base;
            double wheel_radius;
    }




}