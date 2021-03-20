#ifndef CIRCLE_FITTING_INCLUDE_GUARD_HPP
#define CIRCLE_FITTING_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for circle fitting algorithm from ranges

#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include "rigid2d.hpp"
#include <armadillo>


using namespace arma;


namespace rigid2d
{
    /// \brief use ranges to find circle position
    class CircleFitting{
        public:
            /// \brief empty initializer
            CircleFitting();


            /// \brief the wrapper for all the entire method
            /// \param ranges data from laserscan
            /// \return the estimated circle positions
            std::vector<rigid2d::Vector2D> approxCirclePositions(std::vector<double> ranges);

            /// \brief cluster the laserscan data based on distance
            /// \param ranges data from the laserscan
            void clusteringRanges(std::vector<double> ranges);

            /// \brief circle fitting algorithm
            /// \return the estimated circle positions before classification
            std::vector<rigid2d::Vector2D> circleRegression();

            /// \brief classify and remove false positive circle
            /// \param circle_positions the estimated circle positions before classification
            /// \return final version of the circle positions
            std::vector<rigid2d::Vector2D> classifyCircle(std::vector<rigid2d::Vector2D> circle_positions);

            /// \brief get current clusters in ranges
            /// \return point separated into clusters
            std::vector<std::vector<double>> get_point_cluster();

            /// \brief get current clusters in radius
            /// \return current clusters radius
            std::vector<double> get_r_cluster();

            /// \brief modify the cluster coordinate
            /// \param new_xy_cluster the modified version of the cluster xy positions
            void set_xy_cluster(std::vector<std::vector<rigid2d::Vector2D>> new_xy_cluster);


        private:
            bool test_flag;
            std::vector<std::vector<double>> point_cluster; //for range
            std::vector<std::vector<rigid2d::Vector2D>> xy_cluster; //for coordinate
            std::vector<double> r_cluster; //for radius




    };


}


#endif