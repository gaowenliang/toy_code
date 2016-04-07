/*****************************************************************************************

********************************************************************************************/
#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <math.h>

#include <cv.h>
#include <opencv2/opencv.hpp>

#include "ultrasonic_radar/radar_msg.h"

#define Mat Mat
#define pi 3.141592

using namespace cv;
using namespace ros;
using namespace std;

ros::Subscriber radar_sub;

Mat window = Mat::zeros(500, 1000, CV_8UC3);
Mat target = Mat::zeros(500, 1000, CV_8UC3);
bool is_first_on = true;
bool bool_up = true;
int range = 1000;
int rangeTmp = 1000;
int Rmax = 0;
static int frame_num;

ofstream datafile("/home/brain/toy_ros_space/src/ultrasonic_radar/data.txt",ios::out);

void  radar_callback(const ultrasonic_radar::radar_msg::ConstPtr& radar_msg );
void imageInit( );
void clear();

int main(int argc,char **argv)
{
    ros::init(argc, argv, "radar_node");
    ros::NodeHandle nh;

    //Create gui
    cv::namedWindow("RADAR",1);
    createTrackbar( "Range", "RADAR", &range, 4000, 0 );

    if(!datafile)
    {
        cout<<"error !";
    }
    else
    {
        datafile<<"Radar data out:O(∩_∩)O~"<<endl<<endl;
    }

    if(is_first_on == true)
    {
        imageInit();
    }
    radar_sub = nh.subscribe("/radar_pub", 50, radar_callback);

    ros::spin();
    datafile<<"Radar data ALL here."<<endl<<"O(∩_∩)O~";
    datafile.close();
    return 0;
}

void radar_callback(const ultrasonic_radar::radar_msg::ConstPtr& radar_msg )
{
    cout<<endl<<"get a radar frame."<<endl;

    int _range = range;

    float r = radar_msg -> distance;
    float r500 = 500;
    int theta = radar_msg -> angle;

    char num_char[2];
    char r_char[2];
    char theta_char[2];

    sprintf(num_char,"%d",frame_num);
    sprintf(theta_char,"%d",theta);
    sprintf(r_char,"%f",r);

    string r_str = r_char;
    string num_str = num_char;
    string theta_str = theta_char;
    //line_string
    //datafile<<"frame: "<<num_str<<" theta: "<<theta_str<<" r: "<<r_str<<endl;
    datafile<<"r: "<<r_str<<endl;
    frame_num = frame_num +1;

    // swap logic
    if( theta == 0 && bool_up == false )
    {
        bool_up = true;
        datafile<<"Scanning go back."<<endl;
    }
    else if( theta == 180 && bool_up == true )
    {
        bool_up = false;
        clear();
    }

    cout<<"r | "<<r<<" | theta | "<<theta<<endl;

    r = r* 500.0f / _range;
    theta = theta - 90;

    if(_range != rangeTmp)
    {
        clear();
        rangeTmp = _range;
    }

    float x;
    float y;
    x = 500 - r*( sin( theta*pi/180));
    y = 500 - r*( cos( theta*pi/180));
    cout<<"x | "<<x<<" | y | "<<y<<endl;

    float x500;
    float y500;
    x500 = 500 - r500*( sin( theta*pi/180));
    y500 = 500 - r500*( cos( theta*pi/180));

    imageInit();

    circle( target, Point2f(x,y), 2, Scalar(0,255,255), 1, 8);
    line( window, Point2f(x500,y500), Point2f(500,500), Scalar(0,255,0), 3, 8);

    Mat tmp = target + window;

    imshow("RADAR", tmp);
    imwrite("radar.jpg",tmp);
    waitKey(1);

    line( window, Point2f(x500,y500), Point2f(500,500), Scalar(0,0,0), 3, 8);

}


void clear()
{
    Mat tmp = Mat::zeros(500, 1000, CV_8UC3);
    tmp.copyTo(window);
    tmp.copyTo(target);
    imageInit();
    datafile<<"A Scanning DONE."<<endl<<endl;
}

void imageInit( )
{
    string showMsg = "Ultrasonic Radar Info.";

    putText( window, showMsg, Point( 5,30),FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 200, 0) );

    circle( window, Point2f(500,500),   5, Scalar(255,0,255), 4, 8);

    circle( window, Point2f(500,500), 100, Scalar(0,180,0), 1, 8);
    circle( window, Point2f(500,500), 200, Scalar(0,180,0), 1, 8);
    circle( window, Point2f(500,500), 300, Scalar(0,180,0), 2, 8);
    circle( window, Point2f(500,500), 400, Scalar(0,180,0), 1, 8);
    circle( window, Point2f(500,500), 500, Scalar(0,180,0), 2, 8);

    line( window, Point2f(0,500), Point2f(1000,500), Scalar(0,200,0), 1, 8);
    line( window, Point2f(500,0), Point2f(500,500), Scalar(0,200,0), 1, 8);
    line( window, Point2f(147,147), Point2f(500,500), Scalar(0,200,0), 1, 8);
    line( window, Point2f(853,147), Point2f(500,500), Scalar(0,200,0), 1, 8);

}
