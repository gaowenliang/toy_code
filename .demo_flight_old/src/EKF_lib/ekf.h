#include <iostream>
#include <stdio.h>

#include </usr/local/include/eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class EKF
{
public:
    EKF(int _n, int _m);

    void init_filter(int _n, int _m);

    void setQ(MatrixXf _Q);
    void setR(MatrixXf _R);

    void setQ(float _Q);
    void setR(float _R);

    void predict(const VectorXf update_data, const float dt);
    VectorXf correct(const VectorXf error_in);

    int n;  //number of state
    int m;  //number of measurement

    VectorXf X_pre;
    VectorXf X_post;
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
