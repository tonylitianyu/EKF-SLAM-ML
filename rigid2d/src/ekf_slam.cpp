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

rigid2d::EKF_SLAM::EKF_SLAM(int n_measurements){
    n = n_measurements;
    mat S_left_top = zeros<mat>(3,3);
    mat S_right_top = zeros<mat>(3,2*n);
    mat S_left_bot = zeros<mat>(2*n,3);
    mat S_right_bot = eye(2*n, 2*n)*100;

    mat S_top = join_horiz(S_left_top, S_right_top);
    mat S_bot = join_horiz(S_left_bot, S_right_bot);
    sigma = join_vert(S_top, S_bot);



    Q = zeros<mat>(3+2*n, 3+2*n);
    Q(0,0) = 0.0001;
    Q(1,1) = 0.0001;
    Q(2,2) = 0.0001;


    state = zeros<mat>(3+2*n, 1);


    //landmark
    landmark_init_flag = false;


}

void rigid2d::EKF_SLAM::prediction(const rigid2d::Twist2D & twist){

    // mat S_left_top = zeros<mat>(3,3);
    // mat S_right_top = zeros<mat>(3,2*n);
    // mat S_left_bot = zeros<mat>(2*n,3);
    // mat S_right_bot = eye(2*n, 2*n)*10000;

    // mat S_top = join_horiz(S_left_top, S_right_top);
    // mat S_bot = join_horiz(S_left_bot, S_right_bot);
    // sigma = join_vert(S_top, S_bot);


    double dtheta = twist.angular();
    //std::cout << dtheta << std::endl;
    double dx = twist.linearX();
    double dy = 0.0;//twist.linearY();

    double theta = state(0,0);
    double x = state(1,0);
    double y = state(2,0);

    mat update = zeros<mat>(3+2*n,1);
    mat A = zeros<mat>(3+2*n, 3+2*n);

    if (fabs(dtheta) < 0.000001){
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


    state = state + update;

    mat At = eye(size(A)) + A;
    sigma = At*sigma*At.t() + Q;



}

void rigid2d::EKF_SLAM::measurement(mat sensor_reading, std::vector<bool> visible_list, std::vector<bool> known_list){
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

        if (visible_list[i] == false){
            continue;
        }
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
        double esti_ri = sqrt(pow(get_tube_x(i) - x, 2.0)+pow(get_tube_y(i) - y, 2.0));
        double esti_phii = atan2(get_tube_y(i) - y, get_tube_x(i) - x) - theta;
        hj(0,0) = esti_ri;
        hj(1,0) = rigid2d::normalize_angle(esti_phii);


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

        //std::cout << z_measure - hj << std::endl;

        mat z_diff = z_measure - hj;
        z_diff(1,0) = rigid2d::normalize_angle(z_diff(1,0));


        state = state + Ki*z_diff;
        state(0,0) = rigid2d::normalize_angle(state(0,0));



        mat kh = Ki*Hj;
        sigma = (eye(size(kh))-kh)*sigma;

    }


}


void rigid2d::EKF_SLAM::initialize_landmark(rigid2d::Vector2D measure, int i){
    double theta = state(0,0);
    double x = state(1,0);
    double y = state(2,0);
    double sensor_x = measure.x;
    double sensor_y = measure.y;
    double ri = sqrt(pow(sensor_x,2)+pow(sensor_y,2));
    double phii = atan2(sensor_y, sensor_x);
    double mx = x+ri*cos(phii + theta);
    double my = y+ri*sin(phii + theta);

    state(i*2+3,0) = mx;
    state(i*2+3+1,0) = my;
    std::cout << "initialize to x: " << mx << " to y: " << my << std::endl;
}


double rigid2d::EKF_SLAM::calculate_maha_dis(rigid2d::Vector2D jth_measure, int ith_tube){
    int i = ith_tube;
    double theta = state(0,0);
    double x = state(1,0);
    double y = state(2,0);


    //z_measurement
    mat z_measure = zeros<mat>(2,1);

    double sensor_x = jth_measure.x;
    double sensor_y = jth_measure.y;
    double ri = sqrt(pow(sensor_x,2)+pow(sensor_y,2));
    double phii = atan2(sensor_y, sensor_x);

    z_measure(0,0) = ri;
    z_measure(1,0) = phii;



    //zhat
    mat hj = zeros<mat>(2,1);
    double esti_ri = sqrt(pow(get_tube_x(i) - x, 2.0)+pow(get_tube_y(i) - y, 2.0));
    double esti_phii = atan2(get_tube_y(i) - y, get_tube_x(i) - x) - theta;
    hj(0,0) = esti_ri;
    hj(1,0) = rigid2d::normalize_angle(esti_phii);




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
    
    mat psi = Hj*sigma*Hj.t() + R;
    std::cout << "psi " << psi << std::endl;
    mat maha_d_mat = ((z_measure - hj).t()*psi.i()*(z_measure - hj));
    double maha_d = maha_d_mat(0,0);

    
    return maha_d;

    
}

void rigid2d::EKF_SLAM::data_association(std::vector<rigid2d::Vector2D> measures, std::vector<bool> &known_list){


    int known_count = 0;
    for (int i = 0; i < known_list.size(); i++){
        if (known_list[i] == true){
            known_count++;
        }else{
            break;
        }
    }

    std::cout << "=========" << measures.size() << std::endl;
    for (int j = 0; j < measures.size(); j++){
        std::cout << "current measure: " << measures[j] << std::endl;
        double min_maha_dis = 10.0;
        int min_maha_idx = known_count;


        std::cout << known_count << std::endl;

        
        for (int i = 0; i < known_count; i++){
            std::cout << "current tube state x: " << state(i*2+3,0) << " y: " << state(i*2+3+1,0) << std::endl;
            double d = calculate_maha_dis(measures[j], i);
            std::cout << "each d: " << d << std::endl;
            
            if (d < min_maha_dis){
                
                min_maha_dis = d;
                min_maha_idx = i;
            }
            // else{
            //     visible_list[i] = false;
            // }

        }
        std::cout << "min_maha_dis: " << min_maha_dis << std::endl;
        //visible_list[min_maha_idx] = true;
        
        if (min_maha_idx == known_count){
            
            initialize_landmark(measures[j], min_maha_idx);
            std::cout << "Initialize new tube at " << min_maha_idx << " to " << get_tube_x(min_maha_idx) << std::endl;
            
            known_list[known_count] = true;
            known_count++;
            ///test
            min_maha_dis = 0.0;
        }
        if (min_maha_dis < 0.01){
            double theta = state(0,0);
            double x = state(1,0);
            double y = state(2,0);

            //z_measurement
            mat z_measure = zeros<mat>(2,1);

            double sensor_x = measures[j].x;
            double sensor_y = measures[j].y;
            double ri = sqrt(pow(sensor_x,2)+pow(sensor_y,2));
            double phii = atan2(sensor_y, sensor_x);

            z_measure(0,0) = ri;
            z_measure(1,0) = phii;

            

            //z_hat
            mat hj = zeros<mat>(2,1);
            double esti_ri = sqrt(pow(get_tube_x(min_maha_idx) - x, 2.0)+pow(get_tube_y(min_maha_idx) - y, 2.0));
            double esti_phii = atan2(get_tube_y(min_maha_idx) - y, get_tube_x(min_maha_idx) - x) - theta;
            hj(0,0) = esti_ri;
            hj(1,0) = rigid2d::normalize_angle(esti_phii);


            double delta_x = get_tube_x(min_maha_idx) - x;
            double delta_y = get_tube_y(min_maha_idx) - y;
            double d = pow(delta_x,2) + pow(delta_y, 2);

            mat Hj;

            mat H_left = {{0, -delta_x/sqrt(d), -delta_y/sqrt(d)}, {-1, delta_y/d, -delta_x/d}};
            mat H_left_mid = zeros<mat>(2, 2*(min_maha_idx+1-1));
            mat H_right_mid = {{delta_x/sqrt(d), delta_y/sqrt(d)}, {-delta_y/d, delta_x/d}};
            mat H_right = zeros<mat>(2, 2*n - 2*(min_maha_idx+1));
            Hj = join_horiz(H_left, H_left_mid);
            Hj = join_horiz(Hj, H_right_mid);
            Hj = join_horiz(Hj, H_right);

            mat R = zeros<mat>(2,2);

            R(0,0) = 0.01;
            R(1,1) = 0.01;


            mat Ki = sigma*Hj.t()*(Hj*sigma*Hj.t() + R).i();

            //std::cout << z_measure - hj << std::endl;

            mat z_diff = z_measure - hj;
            z_diff(1,0) = rigid2d::normalize_angle(z_diff(1,0));


            state = state + Ki*z_diff;
            state(0,0) = rigid2d::normalize_angle(state(0,0));



            mat kh = Ki*Hj;
            sigma = (eye(size(kh))-kh)*sigma;
        }


        

        
    }

    std::cout << "============" << std::endl;


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

mat rigid2d::EKF_SLAM::getStateLandmark(){
    return state.rows(3,3+2*n-1);
}