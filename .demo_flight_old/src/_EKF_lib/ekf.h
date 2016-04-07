#include <iostream>
#include <stdio.h>

#include </usr/local/include/eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class EKF
{
public:
    EKF(int _n, int _m,
        void (*_f_func)(MatrixXf&, const VectorXf&, const float dt),
        void (*_h_func)(MatrixXf&, const VectorXf&, const VectorXf&),
        MatrixXf _Q, MatrixXf _R);

    void init_filter(int _n, int _m);

    void setQ(MatrixXf _Q);
    void setR(MatrixXf _R);

    void setQ(float _Q);
    void setR(float _R);

    void predict(const VectorXf update_data, const float dt);
    void correct(const VectorXf meas_data);

    int n;  //number of state
    int m;  //number of measurement

    VectorXf X_pre;
    VectorXf X_post;
    MatrixXf P_post;

private:
    VectorXf error_in;

    MatrixXf P_pre;

    MatrixXf F_mat;
    MatrixXf A_mat;
    MatrixXf G_mat;
    MatrixXf Q_mat;

    MatrixXf H_mat;
    MatrixXf R_mat;
    MatrixXf Kg;


    void (*calc_F_function)(MatrixXf&, const VectorXf&, const float dt);
    void (*calc_H_function)(MatrixXf&, const VectorXf&, const VectorXf&);

};
