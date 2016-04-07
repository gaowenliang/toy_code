#include <iostream>
#include <stdio.h>

#include <Eigen/Dense>

#include "math_uilts.h"

using namespace std;
using namespace Eigen;

class EKF
{
public:
    EKF(int _n, int _m);

    Quaterniond q_vb;
    Vector3d p_v;
    Vector3d v_v, v_v_post;
    Vector3d bias_v_b;

    void init_filter(int _n, int _m);

    void setQ(MatrixXf _Q);
    void setR(MatrixXf _R);

    void setQ(float _Q);
    void setR(float _R);

    void predict(const VectorXd update_data, const float dt);
    void measrue_update();
    void correct();

    int n;  //number of state
    int m;  //number of measurement

    bool is_init;

    MatrixXf P_post;

    VectorXf error_in;
    VectorXf error_out;

private:

    MatrixXf P_pre;

    MatrixXf F_mat;
    MatrixXf A_mat;
    MatrixXf G_mat;
    MatrixXf Q_mat;

    MatrixXf H_mat;
    MatrixXf R_mat;
    MatrixXf Kg;

};
