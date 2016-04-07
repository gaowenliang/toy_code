#include <iostream>
#include </usr/local/include/eigen3/Eigen/Dense>
#include </usr/local/include/opencv2/opencv.hpp>
#include </usr/local/include/opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

void printDCM(Matrix3d R)
{
    for(int i = 0;i< 3;i++)
    {
        for(int j = 0;j<3;j++)
            cout<<R.coeff(i,j)<<" ";
        cout<<endl;
    }

}

int main(int argc, char **argv)
{
    Vector4d q0 = Vector4d(0, 0, 0, 1);

    Quaterniond q_eb(1 ,0, 0, 0);
    Quaterniond q00(q0);

    q_eb.normalize();

    Matrix3d R_eb;
    R_eb = q_eb.toRotationMatrix();
    cout<<q_eb.coeffs()<<endl;
    cout<<q00.coeffs()<<endl;

    printDCM(R_eb);


    Mat opencvmat(3,3,CV_32FC1,Scalar(0));
    Matrix3d eigenmat;
    cout<<"opencvMat"<<opencvmat<<endl;


    cv::cv2eigen(opencvmat,eigenmat);
    printDCM(eigenmat);

    return 0;
}


