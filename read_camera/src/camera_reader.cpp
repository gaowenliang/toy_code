#define CAMERA_NUM -1

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
#include <sensor_msgs/Image.h>


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
 */

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_reader");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 2);

    int width = 640;
    int height = 480;
    double exposure;
    double brightness;
    double contrast;
    double saturation;
    double hue;
    double gain;
    double rate;

    nh.param("width",      width,      int(640));
    nh.param("height",     height,     int(480));
    nh.param("exposure",   exposure,   double(0.2));//0.2
    nh.param("brightness", brightness, double(0.5));//0.2
    nh.param("contrast",   contrast,   double(0.6));//0.6
    nh.param("saturation", saturation, double(0.5));//0.2
    nh.param("hue",        hue,        double(0.5));//0.15
    nh.param("gain",       gain,       double(0.5));//0.2
    nh.param("rate",       rate,       double(30));//0.2


    cv_bridge::CvImagePtr cvPtr;
    cv_bridge::CvImage outMsg;

    sensor_msgs::Image img_out;

    ros::Rate loopRate((int)rate);

    Mat view;
    Mat Image;

    //choose the camera by number
    cv::VideoCapture cap(CAMERA_NUM);
    //cap.open(CAMERA_NUM);

    //cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
    /* cap.set(CV_CAP_PROP_FRAME_WIDTH ,width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT ,height);
    cap.set(CV_CAP_PROP_EXPOSURE ,exposure);
    cap.set(CV_CAP_PROP_BRIGHTNESS ,brightness);
    cap.set(CV_CAP_PROP_CONTRAST,contrast);
    cap.set(CV_CAP_PROP_SATURATION,saturation);
    cap.set(CV_CAP_PROP_HUE,hue);
    cap.set(CV_CAP_PROP_GAIN,gain);
    cap.set(CV_CAP_PROP_FPS, rate);*/

    while (nh.ok())
    {
        cap>>view;
        //imshow("Source View", view);
        Image = view;

        ROS_INFO("at time %f", ros::Time::now().toSec());
        std::cout<<"Image Info: "<<"height-"<<Image.rows<<" width-"<<Image.cols<<std::endl;

        outMsg.image = Image;
        outMsg.header.stamp = ros::Time::now();
        outMsg.encoding = "bgr8";

        outMsg.toImageMsg(img_out);
        pub.publish(img_out);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
