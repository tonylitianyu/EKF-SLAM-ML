#include "rigid2d/circle_fitting.hpp"
#include<iostream>




rigid2d::CircleFitting::CircleFitting(){
    test_flag = false;
}

void rigid2d::CircleFitting::clusteringRanges(std::vector<double> ranges){
    point_cluster.clear();
    xy_cluster.clear();
    r_cluster.clear();
    int num_readings = ranges.size();
    double angle_resolution = 2*rigid2d::PI/(double)num_readings;
    double thres = 0.2;


    std::vector<double> curr_cluster;
    std::vector<rigid2d::Vector2D> xy_curr_cluster;

    curr_cluster.push_back(ranges[0]);

    double x0 = ranges[0]*cos(0.0);
    double y0 = ranges[0]*sin(0.0);
    rigid2d::Vector2D xy0 = {x0, y0};
    xy_curr_cluster.push_back(xy0);

    for (int i = 1; i < num_readings; i++){
        if ((fabs(ranges[i] - ranges[i-1]) < thres) && (i != (num_readings - 1))){
            
        }else{
            if (curr_cluster.size() > 6){
                point_cluster.push_back(curr_cluster);
                xy_cluster.push_back(xy_curr_cluster);
            }
            curr_cluster.clear();
            xy_curr_cluster.clear();
        }  
        curr_cluster.push_back(ranges[i]);

        double xi = ranges[i]*cos(rigid2d::normalize_angle(i*angle_resolution));
        double yi = ranges[i]*sin(rigid2d::normalize_angle(i*angle_resolution));
        rigid2d::Vector2D xyi = {xi,yi};
        xy_curr_cluster.push_back(xyi);

        
    }



    //check if the last and the first are the same cluster
    double first_elem_of_first = point_cluster[0][0];
    std::vector<double> last_cluster = point_cluster[point_cluster.size()-1];
    double last_elem_of_last = last_cluster[last_cluster.size()-1];

    std::vector<rigid2d::Vector2D> xy_last_cluster = xy_cluster[xy_cluster.size()-1];
    rigid2d::Vector2D xy_last_elem_of_last = xy_last_cluster[xy_last_cluster.size()-1];


    if (fabs(first_elem_of_first - last_elem_of_last) < thres){
        for (unsigned int k = 0; k < last_cluster.size(); k++){
            point_cluster[0].insert(point_cluster[0].begin(),last_cluster[last_cluster.size() - 1 - k]);
            xy_cluster[0].insert(xy_cluster[0].begin(), xy_last_cluster[xy_last_cluster.size() - 1 - k]);
        }

        point_cluster.pop_back();
        xy_cluster.pop_back();
    }


    if (test_flag){
        int counter = 0;
        for(unsigned int i = 0; i < point_cluster.size(); i++){
            for(unsigned int j = 0; j < point_cluster[i].size(); j++){
                std::cout << point_cluster[i][j] << std::endl;
                std::cout << xy_cluster[i][j] << std::endl;
                counter++;
            }
            std::cout << "===========" << std::endl;
        }
        std::cout << counter << std::endl;
    }
    
    test_flag = false;


}

std::vector<std::vector<double>> rigid2d::CircleFitting::get_point_cluster(){
    return point_cluster;
}

void rigid2d::CircleFitting::set_xy_cluster(std::vector<std::vector<rigid2d::Vector2D>> new_xy_cluster){
    xy_cluster = new_xy_cluster;
}

std::vector<double> rigid2d::CircleFitting::get_r_cluster(){
    return r_cluster;
}

std::vector<rigid2d::Vector2D> rigid2d::CircleFitting::circleRegression(){
    std::vector<rigid2d::Vector2D> circle_positions;
    for (int i = 0; i < xy_cluster.size(); i++){
        //for each cluster

        std::vector<rigid2d::Vector2D> curr_cluster = xy_cluster[i];
        int n_point_in_this_cluster = curr_cluster.size();
        //compute mean
        double x_sum = 0.0;
        double y_sum = 0.0;
        for (int k = 0; k < n_point_in_this_cluster; k++){
            x_sum += curr_cluster[k].x;
            y_sum += curr_cluster[k].y;
        }

        double x_mean = x_sum / (double)n_point_in_this_cluster;
        double y_mean = y_sum / (double)n_point_in_this_cluster;

        //shift
        double z_sum = 0.0;
        for (int j = 0; j < n_point_in_this_cluster; j++){
            curr_cluster[j].x = curr_cluster[j].x - x_mean;
            curr_cluster[j].y = curr_cluster[j].y - y_mean;
            double zi = pow(curr_cluster[j].x, 2.0) + pow(curr_cluster[j].y, 2.0);
            z_sum += zi;
        }

        double z_mean = z_sum / (double)n_point_in_this_cluster;
        mat left = zeros<mat>(n_point_in_this_cluster,1);
        mat left_mid = zeros<mat>(n_point_in_this_cluster,1);
        mat right_mid = zeros<mat>(n_point_in_this_cluster,1);
        mat right = ones<mat>(n_point_in_this_cluster,1);

        for (int m = 0; m < n_point_in_this_cluster; m++){
            left(m,0) = pow(curr_cluster[m].x, 2.0) + pow(curr_cluster[m].y, 2.0);
            left_mid(m,0) = curr_cluster[m].x;
            right_mid(m,0) = curr_cluster[m].y;
        }

        mat left_two = join_horiz(left, left_mid);
        mat right_two = join_horiz(right_mid, right);
        mat big_z = join_horiz(left_two, right_two);

        mat M = (1/n_point_in_this_cluster)*big_z.t()*big_z;

        mat H = zeros<mat>(4,4);
        H(0,0) = 8.0*z_mean;
        H(0,3) = 2.0;
        H(1,1) = 1.0;
        H(2,2) = 1.0;
        H(3,0) = 2.0;

        mat H_inv = zeros<mat>(4,4);
        H_inv(0,3) = 0.5;
        H_inv(1,1) = 1.0;
        H_inv(2,2) = 1.0;
        H_inv(3,0) = 0.5;
        H_inv(3,3) = -2.0*z_mean;


        mat U;
        vec s;
        mat V;

        svd(U,s,V,big_z);


        double smallest_sigma = s(3);

        mat A;
        if (smallest_sigma < 1e-12){
            A = V.col(3);
        }else{
            mat Y = V*diagmat(s)*V.t();
            mat Q = Y*H_inv*Y;

            

            cx_vec eigval;
            cx_mat eigvec;
            eig_gen(eigval, eigvec, Q);


            int smallest_eig_index = 0;
            double smallest_eig_val = 1000.0;

            for (int eig_i = 0; eig_i < 4; eig_i++){
                if(eigval(eig_i).real() > 0){
                    if (eigval(eig_i).real() < smallest_eig_val){
                        smallest_eig_val = eigval(eig_i).real();
                        smallest_eig_index = eig_i;
                    }
                }
            }
            
            mat A_star = zeros<mat>(4,1);
            A_star(0,0) = eigvec.col(smallest_eig_index)(0).real();
            A_star(1,0) = eigvec.col(smallest_eig_index)(1).real();
            A_star(2,0) = eigvec.col(smallest_eig_index)(2).real();
            A_star(3,0) = eigvec.col(smallest_eig_index)(3).real();

            // std::cout << eigvec << std::endl;
            // std::cout << eigvec.col(smallest_eig_index) << std::endl;
            // std::cout << A_star << std::endl;
            // std::cout << "=====" << std::endl;


            A = solve(Y, A_star);
            
        }

        double A1 = A(0,0);
        double A2 = A(1,0);
        double A3 = A(2,0);
        double A4 = A(3,0);

        double a = -A2/(2*A1);
        double b = -A3/(2*A1);
        double R_sqr = (pow(A2,2.0)+pow(A3,2.0)-4*A1*A4)/(4*pow(A1,2.0));

        rigid2d::Vector2D circle_pos = {a+x_mean, b+y_mean};
        circle_positions.push_back(circle_pos);

        
        r_cluster.push_back(sqrt(R_sqr));
    }

    return circle_positions;
}

std::vector<rigid2d::Vector2D> rigid2d::CircleFitting::classifyCircle(std::vector<rigid2d::Vector2D> circle_positions){
    std::vector<bool> is_circle_arr; 
    for (int i = 0; i < xy_cluster.size(); i++){
        is_circle_arr.push_back(false);
    }
    for (int i = 0; i < xy_cluster.size(); i++){
        //for each cluster

        std::vector<rigid2d::Vector2D> curr_cluster = xy_cluster[i];
        int n_point_in_this_cluster = curr_cluster.size();
        rigid2d::Vector2D p1 = curr_cluster[0];  //get first end point
        rigid2d::Vector2D p2 = curr_cluster[n_point_in_this_cluster - 1];  //get second end point

        double sum_angle = 0.0;
        for (int k = 1; k < n_point_in_this_cluster - 1; k++){
            //get angle for p1,p,p2
            rigid2d::Vector2D p = curr_cluster[k];
            rigid2d::Vector2D pp1 = {p1.x - p.x, p1.y - p.y};
            rigid2d::Vector2D pp2 = {p2.x - p.x, p2.y - p.y};

            double top_part = pp1.x*pp2.x + pp1.y*pp2.y;
            double bot_part = sqrt(pow(pp1.x,2.0)+pow(pp1.y,2.0)) * sqrt(pow(pp2.x,2.0)+pow(pp2.y,2.0));
            double angle_rad = acos(top_part/bot_part);

            
            sum_angle += angle_rad;

        }

        double mean_angle = sum_angle/(n_point_in_this_cluster - 2);
        double min_mean = 1.5708;
        double max_mean = 2.3562;
        //std::cout << r_cluster[i] << std::endl;


        //rigid2d::Vector2D a_p_for_radius = curr_cluster[0];
        //double curr_cluster_radius = sqrt(pow(a_p_for_radius.x - circle_positions[i].x,2.0)+pow(a_p_for_radius.y - circle_positions[i].y,2.0));
        if (mean_angle > min_mean && mean_angle < max_mean && r_cluster[i] < 0.2){
            is_circle_arr[i] = true;
        }else{
            is_circle_arr[i] = false;
        }



    }

    
    //std::cout << "=======" << std::endl;
    //remove not circle
    std::vector<rigid2d::Vector2D> clean_circle_positions;
    for (int i = 0; i < xy_cluster.size(); i++){
        if (is_circle_arr[i] == true){
            clean_circle_positions.push_back(circle_positions[i]);
            //std::cout << circle_positions[i] << std::endl;
        }

    }
    //std::cout << "=======" << std::endl;

    return clean_circle_positions;
    
}

std::vector<rigid2d::Vector2D> rigid2d::CircleFitting::approxCirclePositions(std::vector<double> ranges){
    clusteringRanges(ranges);
    std::vector<rigid2d::Vector2D> circle_positions = circleRegression();
    std::vector<rigid2d::Vector2D> clean_circle_positions = classifyCircle(circle_positions);

    return clean_circle_positions;
}