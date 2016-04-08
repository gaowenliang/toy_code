#include "ekf.h"

EKF::EKF(int _n, int _m)
{
    init_filter( _n, _m);

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
    //TODO: for nonlinear model, the predict update maybe not above
    P_pre = F_mat * P_post * F_mat.transpose() + Q_mat;
}

VectorXf EKF::correct(const VectorXf error_in)
{

    //P_pre = F_mat * P_post * F_mat.transpose() + G_mat * Q_mat * G_mat.transpose();

    Kg = P_pre * H_mat.transpose() * (H_mat * P_pre * H_mat.transpose() + R_mat).inverse();


    error_out =  Kg * error_in;

    P_pre = (MatrixXf::Identity(Kg.rows(),H_mat.cols()) - Kg * H_mat) *P_pre;

    X_post = X_pre;
    P_post = P_pre;

    return error_out;
}
