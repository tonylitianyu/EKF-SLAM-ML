#include "catch_ros/catch.hpp"
#include "rigid2d/circle_fitting.hpp"
#include <iostream>
#include <sstream>

using namespace rigid2d;

TEST_CASE( "Clustering", "[circle_fitting]"){


    std::vector<double> ranges{0.464083, 0.450519, 0.441809, 0.435588, 0.431033, 0.427749, 0.42552, 0.4238, 0.424225,
                                    0.42552, 0.427749, 0.431033, 0.435588, 0.441809, 0.450519, 0.464083, 1.01247,
                                    1.01543, 1.01872, 1.02234, 1.0263, 1.03061};
    CircleFitting cf = CircleFitting();

    cf.clusteringRanges(ranges);
    std::vector<std::vector<double>> point_cluster = cf.get_point_cluster();

    REQUIRE( point_cluster.size() == 0 );
    REQUIRE( point_cluster[1][0] == Approx(1.01247) );
    

}

TEST_CASE( "Circle Regression Test 1", "[circle_fitting]"){
    CircleFitting cf = CircleFitting();

    std::vector<Vector2D> xys{Vector2D{1.0,7.0}, Vector2D{2.0, 6.0}, Vector2D{5.0, 8.0}, Vector2D{7.0,7.0}, Vector2D{9.0,5.0}, Vector2D{3.0,7.0}};

    std::vector<std::vector<rigid2d::Vector2D>> test1;
    test1.push_back(xys);

    cf.set_xy_cluster(test1);
    std::vector<rigid2d::Vector2D> circle_positions = cf.circleRegression();

    REQUIRE( circle_positions[0].x == Approx(4.615482));
    REQUIRE( circle_positions[0].y == Approx(2.807354));


    REQUIRE( cf.get_r_cluster()[0] == Approx(4.827575));


}

TEST_CASE( "Circle Regression Test 2", "[circle_fitting]"){
    CircleFitting cf = CircleFitting();

    std::vector<Vector2D> xys{Vector2D{-1.0,0.0}, Vector2D{-0.3, -0.06}, Vector2D{0.3, 0.1}, Vector2D{1.0,0.0}};

    std::vector<std::vector<rigid2d::Vector2D>> test1;
    test1.push_back(xys);

    cf.set_xy_cluster(test1);
    std::vector<rigid2d::Vector2D> circle_positions = cf.circleRegression();

    REQUIRE( circle_positions[0].x == Approx(0.4908357));
    REQUIRE( circle_positions[0].y == Approx(-22.15212));


    REQUIRE( cf.get_r_cluster()[0] == Approx(22.17979));


}


TEST_CASE( "Circle Classification", "[circle_fitting]"){
    std::vector<double> ranges{0.713136, 0.682084, 0.668864, 0.660664, 0.65551, 0.652665, 0.651814, 0.652875,0.655952,0.661391,0.670004,0.684042, 1.01247,
                                    1.01543, 1.01872, 1.02234, 1.0263, 1.03061};
    CircleFitting cf = CircleFitting();

    cf.clusteringRanges(ranges);
    std::vector<rigid2d::Vector2D> circle_positions = cf.circleRegression();

    std::vector<rigid2d::Vector2D> clean_circle_positions = cf.classifyCircle(circle_positions);
    REQUIRE( clean_circle_positions.size() == 0);

}
