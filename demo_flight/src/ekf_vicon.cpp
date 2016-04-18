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

#include "estimate_lib/ekf_estimate.h"

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace message_filters;
using namespace estimate;

ros::Publisher ekf_pub;
ros::Subscriber stick_sub;
ros::Subscriber status_sub;
ros::Subscriber imu_sub;
ros::Subscriber vicon_sub;

ekf_estimate* vicon_fusion;

Quaterniond q_ve;
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

    VectorXd vicon_data(7);
    cout<<vicon_data<<endl;

    //q_vb_vicon
    vicon_data.segment<4>(0)=
            Quaterniond(
                vicon->transform.rotation.w,
                vicon->transform.rotation.x,
                vicon->transform.rotation.y,
                vicon->transform.rotation.z).coeffs();

    // p_v_vicon;
    vicon_data.segment<3>(0+4)=
            Vector3d(
                vicon->transform.translation.x,
                vicon->transform.translation.y,
                vicon->transform.translation.z);
    cout<<"read in data"<<vicon_data<<endl;

    if(vicon_fusion->is_init == false)
    {
        vicon_fusion->is_init = vicon_fusion->init_state(vicon_data);
    }
    else
    {
        vicon_fusion->readin_vicon(vicon_data);
    }
    cout<<"get_position_v"<<vicon_fusion->get_position_v()<<endl;
    cout<<"get_velocity_v"<<vicon_fusion->get_velocity_v()<<endl;
}

void imu_callback(const nav_msgs::OdometryConstPtr& imu)
{
    //ROS_INFO("GET imu MSG");

    VectorXd imu_data(14);

    // q_eb;
    imu_data.segment<4>(0) =
            Quaterniond(
                imu->pose.pose.orientation.w,
                imu->pose.pose.orientation.x,
                imu->pose.pose.orientation.y,
                imu->pose.pose.orientation.z).coeffs();

    // w_b;
    imu_data.segment<3>(0+4+3) =
            Vector3d(
                imu->twist.twist.angular.x,
                imu->twist.twist.angular.y,
                imu->twist.twist.angular.z);

    // v_e;
    imu_data.segment<3>(0+4+3) =
            Vector3d(
                imu->twist.twist.linear.x,
                imu->twist.twist.linear.y,
                imu->twist.twist.linear.z);

    // q_ve
    imu_data.segment<4>(0+4+3+3) =
            q_ve.coeffs();

    vicon_fusion->readin_imu(imu_data,dt);

    get_target_pose(stick_status, flight_status);

    demo_flight::ekf_data ekf_data_out;

    ekf_data_out.header = imu->header;

    Vector3d p_v = vicon_fusion->get_position_v();
    ekf_data_out.position_v.x = p_v(0);
    ekf_data_out.position_v.y = p_v(1);
    ekf_data_out.position_v.z = p_v(2);

    Quaterniond q_vb = vicon_fusion->get_orientation_q_vb();
    ekf_data_out.q_vb.w = q_vb.w();
    ekf_data_out.q_vb.x = q_vb.x();
    ekf_data_out.q_vb.y = q_vb.y();
    ekf_data_out.q_vb.z = q_vb.z();

    Vector3d v_v = vicon_fusion->get_velocity_v();
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
    R_ve<<  0 ,-1 , 0,
           -1 , 0 , 0,
            0 , 0 ,-1;
    q_ve = Quaterniond(R_ve);


    vicon_fusion = new ekf_estimate();

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
