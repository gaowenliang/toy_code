#include <iostream>
#include <stdio.h>

#include <Eigen/Dense>

#include "math_uilts.h"

using namespace std;
using namespace Eigen;

class EKF
{
public:
    EKF(int _n, int _m, float _q, float _r);

    Quaterniond q_vb;
    Vector3d p_v;
    Vector3d v_v, v_v_post;
    Vector3d bias_v_b;

    void init_filter(int _n, int _m);

    void setQ(MatrixXd _Q);
    void setR(MatrixXd _R);

    void setQ(float _Q);
    void setR(float _R);

    void predict(const VectorXd update_data, const float dt);
    void measrue_update();
    void correct();

    int n;  //number of state
    int m;  //number of measurement

    bool is_init;

    MatrixXd P_post;

    VectorXd error_in;
    VectorXd error_out;

private:

    MatrixXd P_pre;

    MatrixXd F_mat;
    MatrixXd A_mat;
    MatrixXd G_mat;
    MatrixXd Q_mat;

    MatrixXd H_mat;
    MatrixXd R_mat;
    MatrixXd Kg;

};
