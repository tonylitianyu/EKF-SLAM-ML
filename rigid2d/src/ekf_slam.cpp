#include "rigid2d/ekf_slam.hpp"
#include<iostream>


 std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }

 double rigid2d::EKF_SLAM::get_tube_x(int m){
     return state(m*2+3,0);
 }

 double rigid2d::EKF_SLAM::get_tube_y(int m){
     return state(m*2+3+1,0);
 }


 rigid2d::EKF_SLAM::EKF_SLAM(){
 }

rigid2d::EKF_SLAM::EKF_SLAM(int n_measurements, std::vector<double> process_noise){
    n = n_measurements;
    mat S_left_top = zeros<mat>(3,3);
    mat S_right_top = zeros<mat>(3,2*n);
    mat S_left_bot = zeros<mat>(2*n,3);
    mat S_right_bot = eye(2*n, 2*n)*10000;

    mat S_top = join_horiz(S_left_top, S_right_top);
    mat S_bot = join_horiz(S_left_bot, S_right_bot);
    sigma = join_vert(S_top, S_bot);



    Q = zeros<mat>(3+2*n, 3+2*n);
    Q(0,0) = process_noise[0];
    Q(1,1) = process_noise[1];
    Q(2,2) = process_noise[2];


    state = zeros<mat>(3+2*n, 1);


    //landmark
    landmark_init_flag = false;


}

void rigid2d::EKF_SLAM::prediction(const rigid2d::Twist2D & twist){
    double dtheta = twist.angular();
    double dx = twist.linearX();
    double dy = 0.0;//twist.linearY();

    double theta = state(0,0);
    double x = state(1,0);
    double y = state(2,0);

    mat update = zeros<mat>(3+2*n,1);
    mat A = zeros<mat>(3+2*n, 3+2*n);

    if (fabs(dtheta) < 0.0001){
        update(0,0) = 0;
        update(1,0) = dx*cos(theta);
        update(2,0) = dx*sin(theta);


        A(1,0) = -dx*sin(theta);
        A(2,0) = dx*cos(theta);

    }else{
        update(0,0) = dtheta;
        update(1,0) = -(dx/dtheta)*sin(theta) + (dx/dtheta)*sin(theta+dtheta);
        update(2,0) = (dx/dtheta)*cos(theta) - (dx/dtheta)*cos(theta+dtheta);

        A(1,0) = -(dx/dtheta)*cos(theta) + (dx/dtheta)*cos(theta+dtheta);
        A(2,0) = -(dx/dtheta)*sin(theta) + (dx/dtheta)*sin(theta+dtheta);

    }


    state = state + update;;

    std::cout << state << std::endl;



    mat At = eye(size(A)) + A;
    sigma = At*sigma*At.t() + Q;

}

void rigid2d::EKF_SLAM::correction(mat sensor_reading){
    double theta = state(0,0);
    double x = state(1,0);
    double y = state(2,0);

    if (landmark_init_flag == false){
        for(int i = 0; i < n; i++){
            double sensor_x = sensor_reading(i*2,0);
            double sensor_y = sensor_reading(i*2+1,0);
            double ri = sqrt(pow(sensor_x,2)+pow(sensor_y,2));
            double phii = atan2(sensor_y, sensor_x);
            double mx = x+ri*cos(phii + theta);
            double my = y+ri*sin(phii + theta);

            state(i*2+3,0) = mx;
            state(i*2+3+1,0) = my;

        }
        
        landmark_init_flag = true;
    }

    

    for(int i = 0; i < n; i++){
        //z_measurement
        mat z_measure = zeros<mat>(2,1);

        double sensor_x = sensor_reading(i*2,0);
        double sensor_y = sensor_reading(i*2+1,0);
        double ri = sqrt(pow(sensor_x,2)+pow(sensor_y,2));
        double phii = atan2(sensor_y, sensor_x);

        z_measure(0,0) = ri;
        z_measure(1,0) = phii;

        
        

        //z_hat
        mat hj = zeros<mat>(2,1);
        double esti_ri = sqrt(pow(get_tube_x(i) - x, 2)+pow(get_tube_y(i) - y, 2));
        double esti_phii = atan2(get_tube_y(i) - y, get_tube_x(i) - x) - theta;
        hj(0,0) = esti_ri;
        hj(1,0) = esti_phii;


        double delta_x = get_tube_x(i) - x;
        double delta_y = get_tube_y(i) - y;
        double d = pow(delta_x,2) + pow(delta_y, 2);

        mat Hj;

        mat H_left = {{0, -delta_x/sqrt(d), -delta_y/sqrt(d)}, {-1, delta_y/d, -delta_x/d}};
        mat H_left_mid = zeros<mat>(2, 2*(i+1-1));
        mat H_right_mid = {{delta_x/sqrt(d), delta_y/sqrt(d)}, {-delta_y/d, delta_x/d}};
        mat H_right = zeros<mat>(2, 2*n - 2*(i+1));
        Hj = join_horiz(H_left, H_left_mid);
        Hj = join_horiz(Hj, H_right_mid);
        Hj = join_horiz(Hj, H_right);

        mat R = zeros<mat>(2,2);

        R(0,0) = 0.01;
        R(1,1) = 0.01;


        mat Ki = sigma*Hj.t()*(Hj*sigma*Hj.t() + R).i();


        state = state + Ki*(z_measure - hj);



        mat kh = Ki*Hj;
        sigma = (eye(size(kh))-kh)*sigma;




    }


}

double rigid2d::EKF_SLAM::getStateX(){
    return state(1,0);
}

double rigid2d::EKF_SLAM::getStateY(){
    return state(2,0);
}

double rigid2d::EKF_SLAM::getStateTheta(){
    return state(0,0);
}