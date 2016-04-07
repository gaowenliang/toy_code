#include <iostream>

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

#include "aruco_lib/aruco.h"
#include "aruco_lib/cvdrawingutils.h"

using namespace std;
using namespace ros;
using namespace cv;
using namespace aruco;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "aruco_test");
    ros::NodeHandle n;

    MarkerDetector detector;
    vector<Marker> markers;

    Mat image;
    image = imread("/home/brain/msckf_vins/src/cv_process/image_sample/hqdefault.jpg");

    detector.detect(image, markers);

    for (int i=0;i<markers.size();i++)
    {
        cout<<markers[i]<<endl;
        markers[i].draw(image,Scalar(0,255,255),2);
    }

    imshow("in",image);
    waitKey(0);

    return 0;
}
