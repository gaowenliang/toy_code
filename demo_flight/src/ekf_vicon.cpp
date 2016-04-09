#include <iostream>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

#include "geometry_msgs/TransformStamped.h"
#include "demo_flight/ekf_data.h"
#include "/home/brain/toy_ros_space/devel/include/demo_flight/ekf_data.h"
#include "demo_flight/move_target.h"
#include "demo_flight/stick_data.h"

#include "EKF_lib/ekf.h"

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher ekf_pub;
ros::Subscriber stick_sub;
ros::Subscriber status_sub;
ros::Subscriber imu_sub;
ros::Subscriber vicon_sub;

EKF* ekf;

Vector3d v_e, v_b, v_v_calc;
Vector3d w_b;
Quaterniond q_eb, q_ve, q_vb;
Matrix3d R_ve;
Matrix3d R_eb;

static float dt;

static float yaw_error_b;
static Vector3d p_error_b;

static int flight_status;
static int stick_status;

#define M100_1_HOME_v_x 0.0f
#define M100_1_HOME_v_y -1.5f
#define M100_1_HOME_v_z 1.2f

#define M100_2_HOME_v_x 0.0f
#define M100_2_HOME_v_y 0.0f
#define M100_2_HOME_v_z 1.2f

#define M100_3_HOME_v_x 0.0f
#define M100_3_HOME_v_y 1.5f
#define M100_3_HOME_v_z 1.2f


void get_target_pose(int stick, int state_flight)
{
    switch (stick)
    {
    case 0:
        if(state_flight == 3)
        {

        }
        break;
    case 1:
        if(state_flight == 1)
        {

        }
        else if(state_flight == 2)
        {

        }
        else if(state_flight == 3)
        {

        }
        break;
    case 2:
        if(state_flight == 3)
        {

        }
        else if(state_flight == 4 )
        {

        }
        else if(state_flight == 5)
        {

        }
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        break;
    case 6:
        break;
    default:
        break;
    }
}

void vicon_callback(const geometry_msgs::TransformStampedConstPtr& vicon)
{
    Quaterniond q_vb_vicon, q_bv_vicon, delta_q;
    Vector3d p_v_vicon;

    //ROS_INFO("GET vicon MSG");
    q_vb_vicon.w() = vicon->transform.rotation.w;
    q_vb_vicon.x() = vicon->transform.rotation.x;
    q_vb_vicon.y() = vicon->transform.rotation.y;
    q_vb_vicon.z() = vicon->transform.rotation.z;

    p_v_vicon(0) = vicon->transform.translation.x;
    p_v_vicon(1) = vicon->transform.translation.y;
    p_v_vicon(2) = vicon->transform.translation.z;

    // orietation error
    q_bv_vicon = q_vb_vicon.conjugate();

    delta_q = q_bv_vicon * ekf->q_vb;

    ekf->error_in.segment<3>(0) = -delta_q.vec();// /delta_q.w();
    ekf->error_in.segment<3>(0+3) = p_v_vicon - ekf->p_v;

    if(ekf->is_init == false)
    {
        ekf->p_v = p_v_vicon;
        ekf->q_vb = q_vb_vicon;
        ekf->is_init = true;
    }
    else
    {
        ekf->H_mat<<
                     Matrix3d::Identity(), Matrix3d::Zero(), Matrix3d::Zero(),
                Matrix3d::Zero(), Matrix3d::Identity(), Matrix3d::Zero();

        ekf->measrue_update();
        ekf->correct();
    }
}

void imu_callback(const nav_msgs::OdometryConstPtr& imu)
{
    //ROS_INFO("GET imu MSG");
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


    if(ekf->is_init == true)
    {
        // imu propagation
        ekf->q_vb = q_ve*q_eb;

        R_eb = q_eb.toRotationMatrix();

        ekf->v_v = R_ve*(v_e - R_eb* ekf->bias_v_b);

        v_v_calc = 0.5f*( ekf->v_v + ekf->v_v_post);

        ekf->p_v = ekf->p_v + v_v_calc* dt;

        ekf->v_v_post = ekf->v_v;

        // ekf predict
        ekf->F_mat<<
                     -crossMat(w_b),   Matrix3d::Zero(), Matrix3d::Zero(),
                Matrix3d::Zero(), Matrix3d::Zero(), -R_ve,
                Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero();

        ekf->G_mat<<
                     Matrix3d::Identity(), Matrix3d::Zero(), Matrix3d::Zero(),
                Matrix3d::Zero(),     -R_ve,            Matrix3d::Zero(),
                Matrix3d::Zero(),     Matrix3d::Zero(), Matrix3d::Identity();

        ekf->predict(w_b, dt);

        get_target_pose(stick_status, flight_status);
    }

    demo_flight::ekf_data ekf_data_out;

    ekf_data_out.header = imu->header;

    ekf_data_out.position_v.x = ekf->p_v(0);
    ekf_data_out.position_v.y = ekf->p_v(1);
    ekf_data_out.position_v.z = ekf->p_v(2);

    ekf_data_out.q_vb.w = ekf->q_vb.w();
    ekf_data_out.q_vb.x = ekf->q_vb.x();
    ekf_data_out.q_vb.y = ekf->q_vb.y();
    ekf_data_out.q_vb.z = ekf->q_vb.z();

    ekf_data_out.velocity_v.x = ekf->v_v(0);
    ekf_data_out.velocity_v.y = ekf->v_v(1);
    ekf_data_out.velocity_v.z = ekf->v_v(2);

    ekf_data_out.position_errore_b.x = p_error_b(0);
    ekf_data_out.position_errore_b.y = p_error_b(1);
    ekf_data_out.position_errore_b.z = p_error_b(2);
    ekf_data_out.yaw_error = yaw_error_b;

    ekf_data_out.flight_status = flight_status;
    ekf_data_out.status = stick_status;
    ekf_data_out.delta_t = dt;

    ekf_pub.publish(ekf_data_out);

}

void stick_callback(const demo_flight::stick_dataConstPtr& stick)
{
    int value_bool = stick->valued;

    if(value_bool == 1)
        stick_status = stick->status;
    else
        stick_status = 0;

    cout<<"receive stick status"<<endl;
}

void status_callback(const std_msgs::UInt8ConstPtr&  status)
{
    flight_status = status->data;

    cout<<"receive flight status"<<endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vicon_ekf");
    ros::NodeHandle nh;

    //TODO: what is q_ve?
    R_ve = q_ve.toRotationMatrix();

    ekf = new EKF(9,6,0.1,0.1);

    stick_sub = nh.subscribe("/stick_data",10,stick_callback);
    status_sub = nh.subscribe("/dji_sdk/flight_status",10,status_callback);

    vicon_sub = nh.subscribe("/vicon/M100_1/M100_1",10,vicon_callback);
    imu_sub = nh.subscribe("/dji_sdk/odometry",10,imu_callback);

    ekf_pub = nh.advertise<demo_flight::ekf_data>("/ekf_state",10);

    //message_filters::Subscriber<geometry_msgs::TransformStamped> vicon_sub(nh, "/vicon/M100_1/M100_1", 1);
    //message_filters::Subscriber<nav_msgs::Odometry> imu_sub(nh, "/dji_sdk/odometry", 1);

    //typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped, nav_msgs::Odometry> SyncPolicy;
    //message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(20), vicon_sub, imu_sub);

    //sync.registerCallback(boost::bind(&callback, _1,_2));

    ros::spin();

    return 0;
}
