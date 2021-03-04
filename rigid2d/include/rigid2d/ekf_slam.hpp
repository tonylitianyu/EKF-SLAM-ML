#ifndef EKF_SLAM_INCLUDE_GUARD_HPP
#define EKF_SLAM_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for extended kalman filter SLAM for diff drive robot.


#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include "rigid2d.hpp"
#include <armadillo>
#include <random>
#include <cmath>

using namespace arma;

namespace rigid2d
{
/// \brief SLAM algorithm calculation
    class EKF_SLAM{
        public:


            EKF_SLAM();


            /// \brief create a ekfslam object
            /// \param wheel_base - the distance between two wheels
            /// \param wheel_radius - the radius of the wheels
            EKF_SLAM(int n_measurements, std::vector<double> process_noise);


            void prediction(const rigid2d::Twist2D & twist);


            void correction(mat sensor_reading);

            double getStateX();
            double getStateY();
            double getStateTheta();

        private:

            mat state;
            mat Q;
            mat sigma;
            int n;
            bool landmark_init_flag;

            double get_tube_x(int m);
            double get_tube_y(int m);



    };

}


#endif

