#include "rigid2d.hpp"
#include<iostream>

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v){
    os << '[' << v.x << ' ' << v.y << ']';
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Vector2D & v){
    
    return is;
}

rigid2d::Transform2D::Transform2D() {
    translation.x = 0.0;
    translation.y = 0.0;
    rotation = 0.0;
}

rigid2d::Transform2D::Transform2D(const Vector2D & trans) {
    translation.x = trans.x;
    translation.y = trans.y;
    rotation = 0.0;
}

rigid2d::Transform2D::Transform2D(double radians) {
    translation.x = 0.0;
    translation.y = 0.0;
    rotation = radians;
}


rigid2d::Transform2D::Transform2D(const Vector2D & trans, double radians) {
    translation.x = trans.x;
    translation.y = trans.y;

    rotation = radians;
}


rigid2d::Vector2D rigid2d::Transform2D::operator()(Vector2D v) const{
    Vector2D res_v = {};
    res_v.x = v.x*cos(rotation) - v.y*sin(rotation) + translation.x;
    res_v.y = v.x*sin(rotation) - v.y*cos(rotation) + translation.y;

    return res_v;
}


rigid2d::Transform2D rigid2d::Transform2D::inv() const{
    Vector2D inv_tran = {-translation.x, -translation.y};
    Transform2D tf = Transform2D(inv_tran, -rotation);
    return tf;
}



rigid2d::Transform2D & rigid2d::Transform2D::operator*=(const Transform2D & rhs){
    double r11 = (cos(rotation)*cos(rhs.theta()))-(sin(rotation)*sin(rhs.theta()));
    double compose_rotation = acos(r11);

    double compose_x = rhs.x()*cos(rotation)-rhs.y()*sin(rotation)+translation.x;
    double compose_y = rhs.x()*sin(rotation)+rhs.y()*cos(rotation)+translation.y;

    translation.x = compose_x;
    translation.y = compose_y;
    rotation = compose_rotation;

    return *this;
}


double rigid2d::Transform2D::x() const{
    return translation.x;
}

double rigid2d::Transform2D::y() const{
    return translation.y;
}

double rigid2d::Transform2D::theta() const{
    return rotation;
}




std::ostream & operator<<(std::ostream & os, const rigid2d::Transform2D & tf){
    return os;
}

std::istream & operator>>(std::istream & is, rigid2d::Transform2D & tf){
    return is;
}

rigid2d::Transform2D operator*(rigid2d::Transform2D lhs, const rigid2d::Transform2D & rhs){
    return lhs;
}
