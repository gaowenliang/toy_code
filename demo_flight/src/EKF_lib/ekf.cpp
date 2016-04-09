#include "ekf.h"

EKF::EKF(int _n, int _m, float _q, float _r)
{
    init_filter( _n, _m);

    setQ(_q);
    setR(_r);
}

void EKF::init_filter(int _n, int _m)
{
    n=_n;
    m=_m;

    error_in = VectorXd::Zero(m);

    P_pre = MatrixXd::Identity(n,n);
    P_post = MatrixXd::Identity(n,n);

    F_mat = MatrixXd::Identity(n,n);
    A_mat = MatrixXd::Zero(n,n);
    //TODO: G and Q maybe not such size
    Q_mat = MatrixXd::Identity(n,n);
    G_mat = MatrixXd::Identity(n,n);

    H_mat = MatrixXd::Zero(m,n);
    R_mat = MatrixXd::Identity(m,m);
    Kg = MatrixXd::Zero(n,m);

    q_vb.setIdentity();
    p_v.setZero();
    v_v.setZero();
    v_v_post.setZero();

    is_init = false;
}

void EKF::setQ(MatrixXd _Q)
{
    Q_mat = _Q;
}

void EKF::setR(MatrixXd _R)
{
    R_mat = _R;
}

void EKF::setQ(float _q)
{
    Q_mat = MatrixXd::Identity(n,n);
    Q_mat = Q_mat*_q;
}

void EKF::setR(float _r)
{
    R_mat = MatrixXd::Identity(m,m);
    R_mat = R_mat*_r;
}

void EKF::predict(const VectorXd update_data, const float dt)
{
    //TODO: for nonlinear model, the predict update maybe not above

    if(is_init == true)
        P_pre = F_mat * P_post * F_mat.transpose() + G_mat*Q_mat*G_mat.transpose()*dt;
}

void EKF::measrue_update()
{
    if(is_init == true)
    {
        //P_pre = F_mat * P_post * F_mat.transpose() + G_mat * Q_mat * G_mat.transpose();

        Kg = P_pre * H_mat.transpose() * (H_mat * P_pre * H_mat.transpose() + R_mat).inverse();

        error_out =  Kg * error_in;

        P_pre = (MatrixXd::Identity(Kg.rows(),H_mat.cols()) - Kg * H_mat) *P_pre;
    }
}

void EKF::correct()
{
    Vector3d delta_theta(  error_out.segment<3>(0) );
    Vector3d delta_bias_v( error_out.segment<3>(0+3) );
    Vector3d delta_p_v(    error_out.segment<3>(0+3+3) );

    quaternion_correct(q_vb, delta_theta);
    bias_v_b += delta_bias_v;
    p_v += delta_p_v;

}
