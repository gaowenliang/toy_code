#include <iostream>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include "geometry_msgs/TransformStamped.h"
#include "demo_flight/ekf_data.h"
#include "demo_flight/move_target.h"

#include "EKF_lib/ekf.h"

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher ekf_pub;

EKF* ekf;
Vector3d p_v;
Vector3d v_e, v_v, v_v_post, v_b, v_v_calc;
Vector3d w_b;
Quaterniond q_eb, q_ve, q_vb, q_vb_vicon;
float dt;

void initialize()
{
    p_v <<0,0,0;
    v_v<<0.0,0.0,0.0;
    v_v_post<< 0.0,0.0,0.0;
}
void callback(const geometry_msgs::TransformStampedConstPtr& vicon, const nav_msgs::OdometryConstPtr& imu)
{
    q_vb_vicon.w() = vicon->transform.rotation.w;
    q_vb_vicon.x() = vicon->transform.rotation.x;
    q_vb_vicon.y() = vicon->transform.rotation.y;
    q_vb_vicon.z() = vicon->transform.rotation.z;

    p_v(0) = vicon->transform.translation.x;
    p_v(1) = vicon->transform.translation.y;
    p_v(2) = vicon->transform.translation.z;

    //ROS_INFO("GET vicon MSG");

    q_eb.w() = imu->pose.pose.orientation.w;
    q_eb.x() = imu->pose.pose.orientation.x;
    q_eb.y() = imu->pose.pose.orientation.y;
    q_eb.z() = imu->pose.pose.orientation.z;

    v_e(0) = imu->twist.twist.linear.x;
    v_e(1) = imu->twist.twist.linear.y;
    v_e(2) = imu->twist.twist.linear.z;

    w_b(0) = imu->twist.twist.angular.x;
    w_b(1) = imu->twist.twist.angular.y;
    w_b(2) = imu->twist.twist.angular.z;

    //ROS_INFO("GET imu MSG");

    q_vb = q_ve*q_eb;

    Matrix3d R_ve = q_ve.toRotationMatrix();

    v_v = R_ve*v_e;

    v_v_calc = 0.5f*(v_v+v_v_post);

    p_v = p_v + v_v_calc* dt;

    v_v_post = v_v;

    q_vb = q_ve* q_eb;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;

    ekf = new EKF(9,6);

    ekf_pub = nh.advertise<demo_flight::ekf_data>("/ekf_state",10);

    message_filters::Subscriber<geometry_msgs::TransformStamped> image_sub(nh, "/vicon/M100_1/M100_1", 1);
    message_filters::Subscriber<nav_msgs::Odometry> imu_sub(nh, "/dji_sdk/odometry", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped, nav_msgs::Odometry> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(20), image_sub, imu_sub);

    sync.registerCallback(boost::bind(&callback, _1,_2));

    ros::spin();

    return 0;
}
