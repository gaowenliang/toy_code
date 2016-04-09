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

Quaterniond  q_ve;
Matrix3d R_ve;

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
    //ROS_INFO("GET vicon MSG");
     Quaterniond q_vb_vicon;
    q_vb_vicon.w() = vicon->transform.rotation.w;
    q_vb_vicon.x() = vicon->transform.rotation.x;
    q_vb_vicon.y() = vicon->transform.rotation.y;
    q_vb_vicon.z() = vicon->transform.rotation.z;

    Vector3d p_v_vicon;
    p_v_vicon(0) = vicon->transform.translation.x;
    p_v_vicon(1) = vicon->transform.translation.y;
    p_v_vicon(2) = vicon->transform.translation.z;

    if(ekf->is_init == false)
    {
        ekf->init_ekf_state(q_vb_vicon, p_v_vicon);
    }

    ekf->measrue_update(q_vb_vicon, p_v_vicon);
    ekf->correct();
}

void imu_callback(const nav_msgs::OdometryConstPtr& imu)
{
    //ROS_INFO("GET imu MSG");
    Quaterniond q_eb;
    q_eb.w() = imu->pose.pose.orientation.w;
    q_eb.x() = imu->pose.pose.orientation.x;
    q_eb.y() = imu->pose.pose.orientation.y;
    q_eb.z() = imu->pose.pose.orientation.z;

    Vector3d v_e;
    v_e(0) = imu->twist.twist.linear.x;
    v_e(1) = imu->twist.twist.linear.y;
    v_e(2) = imu->twist.twist.linear.z;

    Vector3d w_b;
    w_b(0) = imu->twist.twist.angular.x;
    w_b(1) = imu->twist.twist.angular.y;
    w_b(2) = imu->twist.twist.angular.z;

    // imu propagation
    // ekf predict
    ekf->predict(q_eb, w_b, v_e, dt);

    get_target_pose(stick_status, flight_status);

    demo_flight::ekf_data ekf_data_out;

    ekf_data_out.header = imu->header;

    Vector3d p_v = ekf->get_position_v();
    ekf_data_out.position_v.x = p_v(0);
    ekf_data_out.position_v.y = p_v(1);
    ekf_data_out.position_v.z = p_v(2);

    Quaterniond q_vb = ekf->get_orientation_q_vb();
    ekf_data_out.q_vb.w = q_vb.w();
    ekf_data_out.q_vb.x = q_vb.x();
    ekf_data_out.q_vb.y = q_vb.y();
    ekf_data_out.q_vb.z = q_vb.z();

    Vector3d v_v = ekf->get_velocity_v();
    ekf_data_out.velocity_v.x = v_v(0);
    ekf_data_out.velocity_v.y = v_v(1);
    ekf_data_out.velocity_v.z = v_v(2);

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
