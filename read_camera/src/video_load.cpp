//include system in/out libraries
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
//#include <string>

//include ros libraries
#include <ros/ros.h>

//include messege libraries
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"

//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//include ros transport&bridge libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/* this file has the coordinate with opencv photo:
 *  (0,0)+------------------------>u
 *       |
 *       |
 *       |
 *       \/v
 *  the number of distence between the circle and UAV
 *   is depend on the hight of uav
 */

using namespace cv;
using namespace std;

String videoName = "/home/brain/msckf_vins/src/read_camera/video/video_test_web.avi";


int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_load");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 3);

    cv_bridge::CvImagePtr cvPtr;
    cv_bridge::CvImage outMsg;

    ros::Rate loopRate(30);

    Mat view;
    Mat Image;

    geometry_msgs::Point32 msg;
    std::stringstream st;

    //choose the camera by number
k:
    cv::VideoCapture capture(videoName);

    int frame_number = capture.get(CV_CAP_PROP_FRAME_COUNT);

    int count=0;

    while (nh.ok())
    {
        capture>>view;
        //imshow("Source View", view);
        Image = view;

        std::cout<<count<<" VIDEO Image Info: "<<"height-"<<Image.rows<<" width-"<<Image.cols<<std::endl;

        outMsg.image = Image;
        outMsg.header.stamp = ros::Time::now();
        outMsg.encoding = "bgr8";

        pub.publish(outMsg.toImageMsg());
        ros::spinOnce();
        loopRate.sleep();

        count ++;
        if(count == frame_number-1)
        {
            goto k;
        }
    }

    return 0;
}
