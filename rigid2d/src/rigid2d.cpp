#include "rigid2d.hpp"
#include<iostream>

void rigid2d::normalize(Vector2D & v){
    double norm = sqrt(pow(v.x,2)+pow(v.y,2));
    v.x = v.x/norm;
    v.y = v.y/norm;
}


std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v){
    os << '[' << v.x << ' ' << v.y << ']';
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Vector2D & v){
    char bracket = is.peek();
    if (bracket == '['){
        is.ignore();
    }

    is >> v.x;
    is >> v.y;

    return is;
}

//Twist2D Class Implementation Begin

rigid2d::Twist2D::Twist2D(){
    ang = 0.0;
    lin.x = 0.0;
    lin.y = 0.0;
}


rigid2d::Twist2D::Twist2D(double angular, const Vector2D & linear){
    ang = angular;
    lin.x = linear.x;
    lin.y = linear.y;
}


double rigid2d::Twist2D::linearX() const{
    return lin.x;
}


double rigid2d::Twist2D::linearY() const{
    return lin.y;
}


double rigid2d::Twist2D::angular() const{
    return ang;
}



std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Twist2D & ts){
    os << "w (degrees/s): " << rigid2d::rad2deg(ts.angular()) << " vx: " << ts.linearX() <<" vy: " << ts.linearY();
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Twist2D & ts){
    double input_ang;
    double input_vx;
    double input_vy;

    is >> input_ang;
    is >> input_vx;
    is >> input_vy;

    rigid2d::Vector2D input_v = {input_vx,input_vy};

    ts = rigid2d::Twist2D(rigid2d::deg2rad(input_ang), input_v);

    return is;
     
}
//Twist2D Class Implementation End



//Transform2D Class Implementation Begin

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
    res_v.y = v.x*sin(rotation) + v.y*cos(rotation) + translation.y;

    return res_v;
}


rigid2d::Transform2D rigid2d::Transform2D::inv() const{
    double inv_x = -translation.x*cos(rotation)-translation.y*sin(rotation);
    double inv_y = translation.x*sin(rotation)-translation.y*cos(rotation);

    Vector2D inv_tran = {inv_x, inv_y};
    Transform2D tf = Transform2D(inv_tran, -rotation);
    return tf;
}



rigid2d::Transform2D & rigid2d::Transform2D::operator*=(const Transform2D & rhs){
    double r11 = (cos(rotation)*cos(rhs.theta()))-(sin(rotation)*sin(rhs.theta()));
    double r21 = (sin(rotation)*cos(rhs.theta()))+(cos(rotation)*sin(rhs.theta()));
    
    double compose_rotation_cos = acos(r11);
    double compose_rotation_sin = asin(r21);

    double compose_rotation = compose_rotation_cos;
    if ((compose_rotation_cos * compose_rotation_sin) < 0){
        compose_rotation = compose_rotation_sin;
    }

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

rigid2d::Twist2D rigid2d::Transform2D::adjConvert(Twist2D & rhs) const{
    double res_ang = rhs.angular();
    double res_vx = rhs.angular()*translation.y + rhs.linearX()*cos(rotation) - rhs.linearY()*sin(rotation);
    double res_vy = -rhs.angular()*translation.x + rhs.linearX()*sin(rotation) + rhs.linearY()*cos(rotation);

    Vector2D res_vec = {res_vx, res_vy};

    Twist2D tw = Twist2D(res_ang, res_vec);
    
    return tw;
}


//Transform2D Class Implementation End

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Transform2D & tf){


    os << "dtheta (degrees): " << rigid2d::rad2deg(tf.theta()) << " dx: " << tf.x() <<" dy: " << tf.y();
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Transform2D & tf){
    double input_rot;
    double input_x;
    double input_y;

    is >> input_rot;
    is >> input_x;
    is >> input_y;


    rigid2d::Vector2D input_v = {input_x,input_y};

    tf = Transform2D(input_v, rigid2d::deg2rad(input_rot));
    return is;
}

rigid2d::Transform2D rigid2d::operator*(rigid2d::Transform2D lhs, const rigid2d::Transform2D & rhs){
    lhs *= rhs;
    return lhs;
}




