#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

inline Eigen::Matrix<double, 3, 3> hat(const Eigen::Matrix<double, 3, 1> v){

    Eigen::Matrix<double, 3, 3> vhat;

    vhat(0,0) = 0;
    vhat(0,1) = -v[2];
    vhat(0,2) = v[1];
    vhat(1,0) = v[2];
    vhat(1,1) = 0;
    vhat(1,2) = -v[0];
    vhat(2,0) = -v[1];
    vhat(2,1) = v[0];
    vhat(2,2) = 0;

    return vhat;
};

inline Eigen::Matrix<double, 3, 1> vect(const Eigen::Matrix<double, 3, 3> mat){

    Eigen::Matrix<double, 3, 1> vector;
    Eigen::Matrix<double, 3, 3> tmp_mat;

    tmp_mat = (mat-mat.transpose())/0.5;
    vector(0,0) = -tmp_mat(1,2);
    vector(1,0) = tmp_mat(0,2);
    vector(2,0) = -tmp_mat(0,1);
    
    return vector;
};

inline Eigen::Matrix<double, 3, 3> createL(
    const Eigen::Matrix<double, 3, 3> rot_cmd, const Eigen::Matrix<double, 3, 3> rot_curr){

    Eigen::Matrix<double, 3, 3> L;
    Eigen::Matrix<double, 3, 1> nd, sd, ad, n, s, a;
    Eigen::Matrix<double, 3, 3> nd_hat, sd_hat, ad_hat, n_hat, s_hat, a_hat;
    
    nd = rot_cmd.col(0);
    sd = rot_cmd.col(1);
    ad = rot_cmd.col(2);
    n = rot_curr.col(0);
    s = rot_curr.col(1);
    a = rot_curr.col(2);
    nd_hat = hat(nd);
    sd_hat = hat(sd);
    ad_hat = hat(ad);
    n_hat = hat(n);
    s_hat = hat(s);
    a_hat = hat(a);

    L = -(nd_hat*n_hat+sd_hat*s_hat+ad_hat*a_hat)/0.5;
    return L;
};

inline Eigen::Matrix<double, 3, 3> createDotL(
    const Eigen::Matrix<double, 3, 3> rot_cmd, const Eigen::Matrix<double, 3, 3> rot_curr, 
    const Eigen::Matrix<double, 3, 1> w_cmd, const Eigen::Matrix<double, 3, 1> w_curr){

    Eigen::Matrix<double, 3, 3> dotL;
    Eigen::Matrix<double, 3, 1> nd, sd, ad, n, s, a;
    Eigen::Matrix<double, 3, 3> nd_hat, sd_hat, ad_hat, n_hat, s_hat, a_hat, w_hat, wd_hat;
    
    nd = rot_cmd.col(0);
    sd = rot_cmd.col(1);
    ad = rot_cmd.col(2);
    n = rot_curr.col(0);
    s = rot_curr.col(1);
    a = rot_curr.col(2);
    nd_hat = hat(nd);
    sd_hat = hat(sd);
    ad_hat = hat(ad);
    n_hat = hat(n);
    s_hat = hat(s);
    a_hat = hat(a);
    w_hat = hat(w_curr);
    wd_hat = hat(w_cmd);

    dotL = -(hat(wd_hat*nd)*n_hat + hat(wd_hat*sd)*s_hat + hat(wd_hat*ad)*a_hat
            + nd_hat*hat(w_hat*n) + sd_hat*hat(w_hat*s) + ad_hat*hat(w_hat*a))/0.5;

    return dotL;
};

