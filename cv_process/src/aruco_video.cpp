#include <iostream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include </usr/local/include/eigen3/Eigen/Dense>

#include "aruco_lib/aruco.h"
#include "aruco_lib/cvdrawingutils.h"

#include "cv_process/aruco_marker.h"

using namespace std;
using namespace ros;
using namespace cv;
using namespace aruco;
using namespace Eigen;


MarkerDetector detector;
vector<Marker> markers;
CameraParameters camera;

ros::Publisher marker_pub;
ros::Subscriber image_sub;

double marker_size;

void image_callback(const sensor_msgs::Image& img)
{
    // 1. get image from ros
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
    cv::imshow("image source", image_data);

    Mat image;
    image_data.copyTo(image);

    // 2. detect ArUco marker
    detector.detect(image, markers, camera, marker_size, false);

    for (int i=0;i<markers.size();i++)
    {
        cout<<markers[i]<<endl;
        markers[i].draw(image,Scalar(0,255,255),2);

        // 3. marker information process
        Mat R_tc, R_ct;

        // rotation vect v_ct --> R_ct
        Rodrigues(markers[i].Rvec, R_ct);

        // R_tc = R_ct'
        R_tc = R_ct.t();

        Mat T_t_tc(3,1,CV_32FC1);

        // T_t_tc = R_tc *T_c_ct
        T_t_tc = - R_tc *markers[i].Tvec;

        // marker rotation process
        float axis_angle_x,axis_angle_y,axis_angle_z;
        float q_theta;
        float q_ct[4], k_q_ct[3];

        axis_angle_x = markers[i].Rvec.at<float>(0,0);
        axis_angle_y = markers[i].Rvec.at<float>(1,0);
        axis_angle_z = markers[i].Rvec.at<float>(2,0);

        q_theta = sqrt(axis_angle_x*axis_angle_x
                       +axis_angle_y*axis_angle_y
                       +axis_angle_z*axis_angle_z);

        // be sure to normalize quaternion
        k_q_ct[0] = axis_angle_x /q_theta;
        k_q_ct[1] = axis_angle_y /q_theta;
        k_q_ct[2] = axis_angle_z /q_theta;

        q_ct[0] =            cos(0.5f* q_theta);
        q_ct[1] = k_q_ct[0] *sin(0.5f* q_theta);
        q_ct[2] = k_q_ct[1] *sin(0.5f* q_theta);
        q_ct[3] = k_q_ct[2] *sin(0.5f* q_theta);

        // marker position process
        float t_t_tc[4];
        t_t_tc[0] = T_t_tc.at<float>(0,0);
        t_t_tc[1] = T_t_tc.at<float>(1,0);
        t_t_tc[2] = T_t_tc.at<float>(2,0);

        // ros message output
        cv_process::aruco_marker marker_out;

        marker_out.header.stamp = img.header.stamp;
        marker_out.header.frame_id  = "world";

        marker_out.id = markers[i].id;

        marker_out.orientation.w = q_ct[0];
        marker_out.orientation.x = q_ct[1];
        marker_out.orientation.y = q_ct[2];
        marker_out.orientation.z = q_ct[3];

        marker_out.position.x = t_t_tc[0];
        marker_out.position.y = t_t_tc[1];
        marker_out.position.z = t_t_tc[2];

        marker_pub.publish(marker_out);

    }


    imshow("marker detected",image);
    waitKey(0);
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "aruco_video");
    ros::NodeHandle n;

    string camera_calibration;
    string camera_calibration_file = "/home/brain/msckf_vins/src/read_camera/xml/1.xml";

    n.param("marker_size", marker_size, double(0.10));
    n.param("camera_calibration", camera_calibration, camera_calibration_file);

    camera.readFromXMLFile(camera_calibration);

    image_sub  = n.subscribe("/camera/image_raw",10,image_callback);
    marker_pub = n.advertise<cv_process::aruco_marker>("/cv_process/aruco",10);

    ros::spin();

    return 0;
}
