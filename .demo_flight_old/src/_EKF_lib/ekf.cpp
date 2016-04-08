#include "ekf.h"

EKF::EKF(int _n, int _m,
         void (*_f_func)(MatrixXf &, const VectorXf &, const float),
         void (*_h_func)(MatrixXf &, const VectorXf &, const VectorXf &),
         MatrixXf _Q, MatrixXf _R)
{
    init_filter( _n, _m);

    calc_F_function = _f_func;
    calc_H_function = _h_func;

    Q_mat = _Q;
    R_mat = _R;
}

void EKF::init_filter(int _n, int _m)
{
    n=_n;
    m=_m;

    X_pre = VectorXf::Zero(n);
    X_post = VectorXf::Zero(n);

    error_in = VectorXf::Zero(m);

    P_pre = MatrixXf::Identity(n,n);
    P_post = MatrixXf::Identity(n,n);

    F_mat = MatrixXf::Identity(n,n);
    A_mat = MatrixXf::Zero(n,n);
    //TODO: G and Q maybe not such size
    Q_mat = MatrixXf::Identity(n,n);
    G_mat = MatrixXf::Identity(n,n);

    H_mat = MatrixXf::Zero(m,n);
    R_mat = MatrixXf::Identity(m,m);
    Kg = MatrixXf::Zero(n,m);

}

void EKF::setQ(MatrixXf _Q)
{
    Q_mat = _Q;
}

void EKF::setR(MatrixXf _R)
{
    R_mat = _R;
}

void EKF::setQ(float _q)
{
    Q_mat = MatrixXf::Identity(n,n);
    Q_mat = Q_mat*_q;
}

void EKF::setR(float _r)
{
    R_mat = MatrixXf::Identity(m,m);
    R_mat = R_mat*_r;
}

void EKF::predict(const VectorXf update_data, const float dt)
{
    calc_F_function(F_mat,update_data,dt);

    //TODO: for nonlinear model, the predict update maybe not above
    X_pre = F_mat * X_post;
}

void EKF::correct(const VectorXf meas_data)
{
    calc_H_function(H_mat,X_pre,meas_data);

    P_pre = F_mat * P_post * F_mat.transpose() + Q_mat;
    //P_pre = F_mat * P_post * F_mat.transpose() + G_mat * Q_mat * G_mat.transpose();

    Kg = P_pre * H_mat.transpose() * (H_mat * P_pre * H_mat.transpose() + R_mat).inverse();

    error_in = meas_data - H_mat * X_pre;

    X_pre = X_pre + Kg * error_in;

    P_pre = (MatrixXf::Identity(Kg.rows(),H_mat.cols()) - Kg * H_mat) *P_pre;

    X_post = X_pre;
    P_post = P_pre;
}
