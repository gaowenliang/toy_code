#include <iostream>
#include <stdio.h>
#include <string.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#define WIDTH 640
#define HEIGHT 480
#define IMAGE_SIZE (HEIGHT * WIDTH)

using namespace cv;

ros::Subscriber image_sub;

Mat image_get(HEIGHT, WIDTH, CV_8UC1);

int count = 0;

VideoWriter writer("/home/brain/msckf_vins/src/read_camera/video/video_test_web.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30.0, Size(640, 480));

void image_callback(const sensor_msgs::Image& img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat image_data = cv_ptr->image;
    cv::imshow("image", image_data);
    writer << image_data;

    // save a lot of images
    char num_char[2];
    sprintf(num_char,"%d",count);
    string num_str = num_char;
    string Name_string="/home/brain/msckf_vins/src/read_camera/image/image_"+num_str;
    Name_string= Name_string + ".jpg";

    imwrite(Name_string, image_data);

    count++;
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_save");

    ros::NodeHandle n;

    image_sub  = n.subscribe("/camera/image_raw",10,image_callback);

    ros::spin();

    return 0;
}
