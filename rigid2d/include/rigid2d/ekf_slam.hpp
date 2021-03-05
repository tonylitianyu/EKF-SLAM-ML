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

            /// \brief create an empty EKF_SLAM objct, no practical usage
            EKF_SLAM();

            /// \brief create an EKF_SLAM object
            /// \param n_measurements - number of tubes
            EKF_SLAM(int n_measurements);

            /// \breif prediction update stage based on odometry twist
            /// \param twist - current twist from odometry
            void prediction(const rigid2d::Twist2D & twist);

            /// \brief correction update stage based on measurement
            /// \param sensor_reading - the sensor reading of the tube position
            /// \param visible_list - the visibility of each tube
            void measurement(mat sensor_reading, std::vector<bool> visible_list);

            /// \brief get SLAM estimation state in x
            /// \return estimated x position
            double getStateX();

            /// \brief get SLAM estimation state in y
            /// \return estimated y position
            double getStateY();

            /// \brief get SLAM estimation state in orientation
            /// \return estimated orientation
            double getStateTheta();

            /// \brief get SLAM estimation state for landmarks
            /// \return estimated landmarks positions
            mat getStateLandmark();

        private:

            mat state;
            mat Q;
            mat sigma;
            int n;
            bool landmark_init_flag;

            /// \brief get SLAM estimation state for a specific landmark
            /// \param m - the index of the landmark
            /// \return estimated x position of the landmark
            double get_tube_x(int m);

            /// \brief get SLAM estimation state for a specific landmark
            /// \param m - the index of the landmark
            /// \return estimated y position of the landmark
            double get_tube_y(int m);



    };

}


#endif


