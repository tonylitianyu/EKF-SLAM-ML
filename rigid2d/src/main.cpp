#include "../include/rigid2d/rigid2d.hpp"
#include<iostream>

int main() {


    std::cout << "Enter T_ab (theta(in degree) x y): ";
    rigid2d::Transform2D t_ab = rigid2d::Transform2D();
    std::cin >> t_ab;

    std::cout << "Enter T_bc (theta(in degree) x y): ";
    rigid2d::Transform2D t_bc = rigid2d::Transform2D();
    std::cin >> t_bc;

    //Output
    std::cout << '\n' << std::endl;
    std::cout << "Output: \n" << std::endl;

    //T_ab
    std::cout << "T_ab: " << t_ab << std::endl;

    //T_ba
    rigid2d::Transform2D t_ba = t_ab.inv();
    std::cout << "T_ba: " << t_ba << std::endl;

    //T_bc
    std::cout << "T_bc: " << t_bc << std::endl;

    //T_cb
    rigid2d::Transform2D t_cb = t_bc.inv();
    std::cout << "T_cb: " << t_cb << std::endl;

    //T_ac
    rigid2d::Vector2D v_ab = {t_ab.x(),t_ab.y()};
    rigid2d::Transform2D t_ab_temp = rigid2d::Transform2D(v_ab, t_ab.theta());
    t_ab_temp*t_bc;
    rigid2d::Transform2D t_ac = t_ab_temp;
    std::cout << "T_ac: " << t_ac << std::endl;

    //T_ca
    rigid2d::Transform2D t_ca = t_ac.inv();
    std::cout << "T_ca: " << t_ca << std::endl;

    std::cout << std::endl;

    //User enter a vector
    std::cout << "Enter a vector (x y): ";
    rigid2d::Vector2D v;
    std::cin >> v;

    std::cout << "Enter the frame (either a, b, or c): ";
    char frame;
    std::cin >> frame;

    rigid2d::Vector2D v_a;
    rigid2d::Vector2D v_b;
    rigid2d::Vector2D v_c;
    
    switch (frame)
    {
    case 'a':
        v_a = v;
        v_b = t_ba(v_a);
        v_c = t_ca(v_a);
        break;

    case 'b':
        v_b = v;
        v_a = t_ab(v_b);
        v_c = t_cb(v_b);
        break;

    case 'c':
        v_c = v;
        v_a = t_ac(v_c);
        v_b = t_bc(v_c);
        break;
    
    default:
        break;
    }


    std::cout << "Vector in frame a: " << v_a << std::endl;
    std::cout << "Vector in frame b: " << v_b << std::endl;
    std::cout << "Vector in frame c: " << v_c << std::endl;

    std::cout << std::endl;


    //Twist

    //User enter a twist
    std::cout << "Enter a twist (w(in degree/s) vx vy) in the same frame: ";
    rigid2d::Twist2D ts;
    std::cin >> ts;
    

    rigid2d::Twist2D ts_a;
    rigid2d::Twist2D ts_b;
    rigid2d::Twist2D ts_c;
    
    switch (frame)
    {
    case 'a':
        ts_a = ts;
        ts_b = t_ba.adjConvert(ts_a);
        ts_c = t_ca.adjConvert(ts_a);
        break;

    case 'b':
        ts_b = ts;
        ts_a = t_ab.adjConvert(ts_b);
        ts_c = t_cb.adjConvert(ts_b);
        break;

    case 'c':
        ts_c = ts;
        ts_a = t_ac.adjConvert(ts_c);
        ts_b = t_bc.adjConvert(ts_c);
        break;
    
    default:
        break;
    }

    std::cout << "Twist in frame a: " << ts_a << std::endl;
    std::cout << "Twist in frame b: " << ts_b << std::endl;
    std::cout << "Twist in frame c: " << ts_c << std::endl;

    std::cout << std::endl;

    return 0;
}