#include "../include/rigid2d/rigid2d.hpp"
#include<iostream>

rigid2d::Vector2D::Vector2D(){
    x = 0.0;
    y = 0.0;
}


rigid2d::Vector2D::Vector2D(double x_val, double y_val){
    x = x_val;
    y = y_val;
}


rigid2d::Vector2D& rigid2d::Vector2D::operator+=(const rigid2d::Vector2D& rhs)
{
    x += rhs.x;
    y += rhs.y;
    return *this;
}

rigid2d::Vector2D operator+(rigid2d::Vector2D lhs, const rigid2d::Vector2D& rhs)
{
    lhs += rhs;
    return lhs;
}

rigid2d::Vector2D& rigid2d::Vector2D::operator-=(const rigid2d::Vector2D& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

rigid2d::Vector2D operator-(rigid2d::Vector2D lhs, const rigid2d::Vector2D& rhs)
{
    lhs -= rhs;
    return lhs;
}

rigid2d::Vector2D& rigid2d::Vector2D::operator*=(const int scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}

rigid2d::Vector2D operator*(const int scalar, rigid2d::Vector2D rhs)
{
    rhs *= scalar;
    return rhs;
}

rigid2d::Vector2D operator*(rigid2d::Vector2D lhs, const int scalar)
{
    lhs *= scalar;
    return lhs;
}

double rigid2d::Vector2D::magnitude()
{
    return sqrt(pow(x,2)+pow(y,2));
}

double rigid2d::Vector2D::angle()
{
    return atan2(y,x);
}

void rigid2d::normalize(Vector2D & v)
{
    double norm = sqrt(pow(v.x,2)+pow(v.y,2));
    v.x = v.x/norm;
    v.y = v.y/norm;
}


std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v)
{
    os << '[' << v.x << ' ' << v.y << ']';
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Vector2D & v)
{
    char bracket = is.peek();
    if (bracket == '['){
        is.ignore();
    }

    is >> v.x;
    is >> v.y;

    return is;
}

//Twist2D Class Implementation Begin

rigid2d::Twist2D::Twist2D()
{
    ang = 0.0;
    lin.x = 0.0;
    lin.y = 0.0;
}


rigid2d::Twist2D::Twist2D(double angular, const Vector2D & linear)
{
    ang = angular;
    lin.x = linear.x;
    lin.y = linear.y;
}


double rigid2d::Twist2D::linearX() const
{
    return lin.x;
}


double rigid2d::Twist2D::linearY() const
{
    return lin.y;
}


double rigid2d::Twist2D::angular() const
{
    return ang;
}



std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Twist2D & ts)
{
    os << "w (degrees/s): " << rigid2d::rad2deg(ts.angular()) << " vx: " << ts.linearX() <<" vy: " << ts.linearY();
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Twist2D & ts)
{
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

rigid2d::Transform2D::Transform2D() 
{
    translation.x = 0.0;
    translation.y = 0.0;
    rotation = 0.0;
}

rigid2d::Transform2D::Transform2D(const Vector2D & trans) 
{
    translation.x = trans.x;
    translation.y = trans.y;
    rotation = 0.0;
}

rigid2d::Transform2D::Transform2D(double radians) 
{
    translation.x = 0.0;
    translation.y = 0.0;
    rotation = radians;
}


rigid2d::Transform2D::Transform2D(const Vector2D & trans, double radians) 
{
    translation.x = trans.x;
    translation.y = trans.y;

    rotation = radians;
}


rigid2d::Vector2D rigid2d::Transform2D::operator()(Vector2D v) const
{
    Vector2D res_v = {};
    res_v.x = v.x*cos(rotation) - v.y*sin(rotation) + translation.x;
    res_v.y = v.x*sin(rotation) + v.y*cos(rotation) + translation.y;

    return res_v;
}


rigid2d::Transform2D rigid2d::Transform2D::inv() const
{
    double inv_x = -translation.x*cos(rotation)-translation.y*sin(rotation);
    double inv_y = translation.x*sin(rotation)-translation.y*cos(rotation);

    Vector2D inv_tran = {inv_x, inv_y};
    Transform2D tf = Transform2D(inv_tran, -rotation);
    return tf;
}



rigid2d::Transform2D & rigid2d::Transform2D::operator*=(const Transform2D & rhs)
{
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


double rigid2d::Transform2D::x() const
{
    return translation.x;
}

double rigid2d::Transform2D::y() const
{
    return translation.y;
}

double rigid2d::Transform2D::theta() const
{
    return rotation;
}

rigid2d::Twist2D rigid2d::Transform2D::adjConvert(Twist2D & rhs) const
{
    double res_ang = rhs.angular();
    double res_vx = rhs.angular()*translation.y + rhs.linearX()*cos(rotation) - rhs.linearY()*sin(rotation);
    double res_vy = -rhs.angular()*translation.x + rhs.linearX()*sin(rotation) + rhs.linearY()*cos(rotation);

    Vector2D res_vec = {res_vx, res_vy};

    Twist2D tw = Twist2D(res_ang, res_vec);
    
    return tw;
}


//Transform2D Class Implementation End

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Transform2D & tf)
{


    os << "dtheta (degrees): " << rigid2d::rad2deg(tf.theta()) << " dx: " << tf.x() <<" dy: " << tf.y();
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Transform2D & tf)
{
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

rigid2d::Transform2D rigid2d::operator*(rigid2d::Transform2D lhs, const rigid2d::Transform2D & rhs)
{
    lhs *= rhs;
    return lhs;
}


rigid2d::Transform2D rigid2d::integrateTwist(const rigid2d::Twist2D& ts)
{
    Transform2D t_bbq;
    if (fabs(ts.angular()) > 0.01){
        double xs = ts.linearY()/ts.angular();
        double ys = -ts.linearX()/ts.angular();

        Vector2D p_sb = Vector2D(xs,ys);
        Transform2D t_sb = Transform2D(p_sb);
        Transform2D t_bs = t_sb.inv();

        Vector2D p_sqbq = Vector2D(t_sb.x(), t_sb.y());
        Transform2D t_sqbq = Transform2D(p_sqbq, t_sb.theta());

        Transform2D t_ssq = Transform2D(ts.angular());

        Transform2D t_bbq_1 = t_ssq*t_sqbq;

        t_bbq = t_bs*t_bbq_1;
        
    }else{
        Vector2D p_bbq = Vector2D(ts.linearX(),ts.linearY());
        t_bbq = Transform2D(p_bbq);

    }

    return t_bbq;


}

//source: https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees
double rigid2d::normalize_angle(double rad){
    
    double reduced_ang = std::fmod(rad,(2*PI));
    double ang = std::fmod((reduced_ang + (2*PI)),(2*PI));
    if (ang > PI){
        ang = ang - (2*PI);
    }

    return ang;
}

