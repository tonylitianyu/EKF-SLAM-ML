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
            CircleFitting();

            std::vector<rigid2d::Vector2D> approxCirclePositions(std::vector<double> ranges);

        private:
            bool test_flag;
            std::vector<std::vector<double>> point_cluster; //for range
            std::vector<std::vector<rigid2d::Vector2D>> xy_cluster; //for coordinate


            void clusteringRanges(std::vector<double> ranges);
            std::vector<rigid2d::Vector2D> circleRegression();
            std::vector<rigid2d::Vector2D> classifyCircle(std::vector<rigid2d::Vector2D> circle_positions);

    };


}


#endif