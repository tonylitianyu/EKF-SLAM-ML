#include "catch_ros/catch.hpp"
#include "../include/rigid2d/rigid2d.hpp"
#include "../include/rigid2d/diff_drive.hpp"
#include <iostream>
#include <sstream>


TEST_CASE( "Vector Normalization", "[vector2d]"){
    rigid2d::Vector2D v = {3.0, 4.0};
    rigid2d::normalize(v);

    REQUIRE( v.x == Approx(0.6) );
    REQUIRE( v.y == Approx(0.8) );
}

TEST_CASE( "Vector Output Stream", "[iostream]"){
    rigid2d::Vector2D v = {3.5, 4.0};
    std::ostringstream ot;
    ot << v;
    REQUIRE( ot.str() == "[3.5 4]");

}

TEST_CASE( "Vector Input Stream", "[iostream]"){
    rigid2d::Vector2D v;
    rigid2d::Vector2D v_b;


    std::string input = "3.5 4.0";
    std::string input_bracket = "[3.5 5.0]";

    std::istringstream is (input);
    std::istringstream is_b (input_bracket);

    is >> v;
    is_b >> v_b;

    REQUIRE( v.x == Approx(3.5) );
    REQUIRE( v.y == Approx(4.0) );
    REQUIRE( v_b.x == Approx(3.5) );
    REQUIRE( v_b.y == Approx(5.0) );

}


TEST_CASE( "Transform2D Identity Constructor", "[Transform2D]"){
    rigid2d::Transform2D tf = rigid2d::Transform2D();
    REQUIRE( tf.theta() == Approx(0.0) );
    REQUIRE( tf.x() == Approx(0.0) );
    REQUIRE( tf.y() == Approx(0.0) );

}

TEST_CASE( "Transform2D Constructor with Translation", "[Transform2D]"){
    rigid2d::Vector2D v = {1.5, 1.5};
    rigid2d::Transform2D tf = rigid2d::Transform2D(v);

    REQUIRE( tf.theta() == Approx(0.0) );
    REQUIRE( tf.x() == Approx(1.5) );
    REQUIRE( tf.y() == Approx(1.5) );

}

TEST_CASE( "Transform2D Constructor with Rotation", "[Transform2D]"){
    double radians = 0.5;
    rigid2d::Transform2D tf = rigid2d::Transform2D(radians);

    REQUIRE( tf.theta() == Approx(0.5) );
    REQUIRE( tf.x() == Approx(0.0) );
    REQUIRE( tf.y() == Approx(0.0) );

}

TEST_CASE( "Transform2D Constructor with Translation and Rotation", "[Transform2D]"){
    rigid2d::Vector2D v = {1.5, 1.5};
    double radians = 0.5;
    rigid2d::Transform2D tf = rigid2d::Transform2D(v, radians);

    REQUIRE( tf.theta() == Approx(0.5) );
    REQUIRE( tf.x() == Approx(1.5) );
    REQUIRE( tf.y() == Approx(1.5) );

}

TEST_CASE( "Apply Transformation to a Vector", "[Transform2D]"){
    rigid2d::Vector2D v_a = {2.0, 3.0};

    rigid2d::Vector2D v = {1.5, 1.5};
    double radians = 0.5;
    rigid2d::Transform2D tf_ba = rigid2d::Transform2D(v, radians);

    rigid2d::Vector2D v_b = tf_ba(v_a);

    REQUIRE( v_b.x == Approx(1.816889));
    REQUIRE( v_b.y == Approx(5.091599));

}


TEST_CASE( "Inverse the Transformation", "[Transform2D]" ){
    rigid2d::Vector2D v = {1.5, 1.5};
    double radians = 0.5;
    rigid2d::Transform2D tf = rigid2d::Transform2D(v, radians);

    rigid2d::Transform2D inv_tf = tf.inv();

    REQUIRE( inv_tf.theta() == Approx(-0.5) );
    REQUIRE( inv_tf.x() == Approx(-2.03551215) );
    REQUIRE( inv_tf.y() == Approx(-0.59723553) );
}

TEST_CASE( "Compose Transformations with *=", "[Transform2D]"){
    rigid2d::Vector2D v_ab = {1.5, 1.5};
    double radians_ab = 0.5;
    rigid2d::Transform2D tf_ab_temp = rigid2d::Transform2D(v_ab, radians_ab);

    rigid2d::Vector2D v_bc = {2.5, 2.5};
    double radians_bc = -1.1;
    rigid2d::Transform2D tf_bc = rigid2d::Transform2D(v_bc, radians_bc);

    tf_ab_temp*=tf_bc;
    rigid2d::Transform2D tf_ac = tf_ab_temp;

    REQUIRE( tf_ac.theta() == Approx(-0.6));
    REQUIRE( tf_ac.x() == Approx(2.49539256));
    REQUIRE( tf_ac.y() == Approx(4.89252025));


}


TEST_CASE( "Get x Displacement", "[Transform2D]"){
    rigid2d::Vector2D v = {1.5, 1.6};
    double radians = 0.5;
    rigid2d::Transform2D tf = rigid2d::Transform2D(v, radians);

    REQUIRE( tf.x() == Approx(1.5));
}

TEST_CASE( "Get y Displacement", "[Transform2D]"){
    rigid2d::Vector2D v = {1.5, 1.6};
    double radians = 0.5;
    rigid2d::Transform2D tf = rigid2d::Transform2D(v, radians);

    REQUIRE( tf.y() == Approx(1.6));
}

TEST_CASE( "Get theta rotation", "[Transform2D]"){
    rigid2d::Vector2D v = {1.5, 1.6};
    double radians = 0.5;
    rigid2d::Transform2D tf = rigid2d::Transform2D(v, radians);

    REQUIRE( tf.theta() == Approx(0.5));
}

TEST_CASE( "Convert a Twist to a Different Frame Using the Adjoint", "[Transform2D"){
    rigid2d::Vector2D lin = {0.5, 0.6};
    double ang = 0.1;
    rigid2d::Twist2D tw = rigid2d::Twist2D(ang, lin);

    rigid2d::Vector2D v = {1.5, 1.5};
    double radians = 0.5;
    rigid2d::Transform2D tf = rigid2d::Transform2D(v, radians);

    rigid2d::Twist2D tw_b = tf.adjConvert(tw);


    REQUIRE( tw_b.angular() == Approx(0.1));
    REQUIRE( tw_b.linearX() == Approx(0.30113596));
    REQUIRE( tw_b.linearY() == Approx(0.61626231));

}


TEST_CASE( "Transform2D Ouput Stream", "[iostream]"){
    rigid2d::Vector2D v = {1.5, 1.5};
    double radians = 0.5;
    rigid2d::Transform2D tf = rigid2d::Transform2D(v, radians);
    std::ostringstream ot;
    ot << tf;
    REQUIRE( ot.str() == "dtheta (degrees): 28.6479 dx: 1.5 dy: 1.5");
}

TEST_CASE( "Transform2D Input Stream", "[iostream]"){
    rigid2d::Transform2D tf;

    std::string input = "30 3.0 4.0";

    std::istringstream is (input);

    is >> tf;

    REQUIRE( tf.theta() == Approx(0.5236) );
    REQUIRE( tf.x() == Approx(3.0) );
    REQUIRE( tf.y() == Approx(4.0) );

}

TEST_CASE( "Compose Transformations with *", "[Transform2D]"){
    rigid2d::Vector2D v_ab = {1.5, 1.5};
    double radians_ab = 0.5;
    rigid2d::Transform2D tf_ab_temp = rigid2d::Transform2D(v_ab, radians_ab);

    rigid2d::Vector2D v_bc = {2.5, 2.5};
    double radians_bc = 1.1;
    rigid2d::Transform2D tf_bc = rigid2d::Transform2D(v_bc, radians_bc);


    rigid2d::Transform2D tf_ac = tf_ab_temp*tf_bc;

    REQUIRE( tf_ac.theta() == Approx(1.6));
    REQUIRE( tf_ac.x() == Approx(2.49539256));
    REQUIRE( tf_ac.y() == Approx(4.89252025));
}


TEST_CASE( "Twist2D Constructor","[Twist2D]" ){
    rigid2d::Twist2D tw = rigid2d::Twist2D();
    REQUIRE(tw.angular() == Approx(0.0));
    REQUIRE(tw.linearX() == Approx(0.0));
    REQUIRE(tw.linearY() == Approx(0.0));

}

TEST_CASE( "Twist2D Constructor with angular and linear velocity","[Twist2D]" ){
    rigid2d::Vector2D lin = {0.5, 0.6};
    double ang = 0.1;
    rigid2d::Twist2D tw = rigid2d::Twist2D(ang, lin);
    REQUIRE(tw.angular() == Approx(0.1));
    REQUIRE(tw.linearX() == Approx(0.5));
    REQUIRE(tw.linearY() == Approx(0.6));

}

TEST_CASE( "Twist2D get angular velocity","[Twist2D]" ){
    rigid2d::Vector2D lin = {0.5, 0.6};
    double ang = 0.1;
    rigid2d::Twist2D tw = rigid2d::Twist2D(ang, lin);

    REQUIRE(tw.angular() == Approx(0.1));
}

TEST_CASE( "Twist2D get x component of linear velociry","[Twist2D]" ){
    rigid2d::Vector2D lin = {0.5, 0.6};
    double ang = 0.1;
    rigid2d::Twist2D tw = rigid2d::Twist2D(ang, lin);

    REQUIRE(tw.linearX() == Approx(0.5));
}

TEST_CASE( "Twist2D get y component of linear velociry","[Twist2D]" ){
    rigid2d::Vector2D lin = {0.5, 0.6};
    double ang = 0.1;
    rigid2d::Twist2D tw = rigid2d::Twist2D(ang, lin);

    REQUIRE(tw.linearY() == Approx(0.6));
}


TEST_CASE( "Twist2D Ouput Stream", "[iostream]"){
    rigid2d::Vector2D lin = {0.5, 0.6};
    double ang = 0.1;
    rigid2d::Twist2D tw = rigid2d::Twist2D(ang, lin);
    std::ostringstream ot;
    ot << tw;
    REQUIRE( ot.str() == "w (degrees/s): 5.72958 vx: 0.5 vy: 0.6");
}

TEST_CASE( "Twist2D Input Stream", "[iostream]"){
    rigid2d::Twist2D tw;

    std::string input = "30 0.5 0.6";

    std::istringstream is (input);

    is >> tw;

    REQUIRE( tw.angular() == Approx(0.5236) );
    REQUIRE( tw.linearX() == Approx(0.5) );
    REQUIRE( tw.linearY() == Approx(0.6) );

}


TEST_CASE( "Integrate a Twist with Pure Translation", "[Integration]"){
    rigid2d::Vector2D tran = rigid2d::Vector2D(1.0, 2.0);
    rigid2d::Twist2D ts = rigid2d::Twist2D(0.0, tran);

    rigid2d::Transform2D t_bbq = rigid2d::integrateTwist(ts);
    REQUIRE( t_bbq.theta() == Approx(0.0) );
    REQUIRE( t_bbq.x() == Approx(1.0) );
    REQUIRE( t_bbq.y() == Approx(2.0) );

}

TEST_CASE( "Integrate a Twist with Pure Rotation", "[Integration]"){
    rigid2d::Vector2D tran = rigid2d::Vector2D(0.0, 0.0);
    rigid2d::Twist2D ts = rigid2d::Twist2D(0.5, tran);

    rigid2d::Transform2D t_bbq = rigid2d::integrateTwist(ts);
    REQUIRE( t_bbq.theta() == Approx(0.5) );
    REQUIRE( t_bbq.x() == Approx(0.0) );
    REQUIRE( t_bbq.y() == Approx(0.0) );

}


TEST_CASE( "Integrate a Twist with Simultaneous Translation and Rotation", "[Integration]"){
    rigid2d::Vector2D tran = rigid2d::Vector2D(1.0, 2.2);
    rigid2d::Twist2D ts = rigid2d::Twist2D(0.5, tran);

    rigid2d::Transform2D t_bbq = rigid2d::integrateTwist(ts);
    REQUIRE( t_bbq.theta() == Approx(0.5) );
    REQUIRE( t_bbq.x() == Approx(0.4202143) );
    REQUIRE( t_bbq.y() == Approx(2.3543072) );


}



TEST_CASE( "Normalizing Angle", "[Integration]"){
    double simple = rigid2d::normalize_angle(rigid2d::deg2rad(30));
    double out = rigid2d::normalize_angle(rigid2d::deg2rad(230));
    double negative = rigid2d::normalize_angle(rigid2d::deg2rad(-330));

    REQUIRE( simple == Approx(0.523599) );
    REQUIRE( out == Approx(-2.26893) );
    REQUIRE( negative == Approx(0.523599) );   
    
}


TEST_CASE( "Update Pose Forward", "[Odometry]"){
    double left_angle = 0.5;
    double right_angle = 0.5;
    
    rigid2d::DiffDrive dd = rigid2d::DiffDrive(0.2, 0.01);

    dd.updatePose(left_angle, right_angle);

    REQUIRE( dd.getPosition().x == Approx(0.005));
    REQUIRE( dd.getPosition().y == Approx(0.0));
}


TEST_CASE( "Update Pose Backward", "[Odometry]"){
    double left_angle = -0.5;
    double right_angle = -0.5;
    
    rigid2d::DiffDrive dd = rigid2d::DiffDrive(0.2, 0.01);

    dd.updatePose(left_angle, right_angle);

    REQUIRE( dd.getPosition().x == Approx(-0.005));
    REQUIRE( dd.getPosition().y == Approx(0.0));
}

TEST_CASE( "Update Pose Rotation in Position", "[Odometry]"){
    double left_angle = -0.5;
    double right_angle = 0.5;
    
    rigid2d::DiffDrive dd = rigid2d::DiffDrive(0.2, 0.01);

    dd.updatePose(left_angle, right_angle);

    REQUIRE( dd.getPosition().x == Approx(0.0));
    REQUIRE( dd.getPosition().y == Approx(0.0));
}

TEST_CASE( "Update Pose Rotation 90 degree ", "[Odometry]"){
    double left_angle = 0.0;
    double right_angle = 2*3.1415926;
    
    rigid2d::DiffDrive dd = rigid2d::DiffDrive(0.2, 0.05);

    dd.updatePose(left_angle, right_angle);
    REQUIRE( dd.getTheta() == Approx(1.5708) );
    REQUIRE( dd.getPosition().x == Approx(0.1) );
    REQUIRE( dd.getPosition().y == Approx(0.1) );
}