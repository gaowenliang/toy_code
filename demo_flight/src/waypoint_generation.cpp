#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <ros/ros.h>

//#include <Eigen/Dense>

#include "PID_lib/pid.h"

#include "geometry_msgs/TransformStamped.h"
//#include "demo_flight/ekf_data.h"
#include "demo_flight/pid_ctrl_data.h"
#include "demo_flight/move_target.h"
#include "demo_flight/stick_data.h"
#include "math_lib/math_lib.h"

//#include "stick_data.h"
//#include "move_target.h"
//#include "ekf_data.h"
//#include "pid_ctrl_data.h"

#define home_M1_x -0.5
#define home_M1_y -1.0
#define home_M2_x -0.5
#define home_M2_y -1.0
#define home_M3_x -0.5
#define home_M3_y 2.5
#define home_z 1.0

using namespace std;
using namespace ros;

ros::Publisher waypoint_pub;
ros::Subscriber stick_sub;
ros::Subscriber vicon_sub;

//! @todo add these
float position_1[3];
float position_2[3];
float position_3[3];
vector3f distance_v;
float distance_error;

void stick_callback(const demo_flight::stick_data& stick);
void vicon_callback(const geometry_msgs::TransformStamped& vicon_data);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_generation");
    ros::NodeHandle nh("~");

    vicon_sub = nh.subscribe("/vicon/M100_1/M100_1", 5, vicon_callback);

    waypoint_pub = nh.advertise<demo_flight::move_target>("/move_target", 4);
    stick_sub = nh.subscribe("/stick_data", 4, stick_callback);

    ros::spin();

    return 0;
}

void stick_callback(const demo_flight::stick_data& stick)
{
    int status = stick.status;

    cout << "status " << status << endl;

    static ros::Time msRecord = ros::Time::now();
    static int statusRecord = -1;

    static float target_1_home[3] = { home_M1_x, home_M1_y, home_z };
    static float target_2_home[3] = { home_M2_x, home_M2_y, home_z };
    static float target_3_home[3] = { home_M3_x, home_M3_y, home_z };

    float target_1[3];
    float target_2[3];
    float target_3[3];

    for (int i = 0; i < 3; ++i)
    {
        target_2[i] = position_2[i];
        target_1[i] = position_1[i];
        target_3[i] = position_3[i];
    }

    // printf("time: %d %d \n",stick.header.stamp.sec,stick.header.stamp.nsec);

    cout << "status " << status << endl;
    cout << "statusRecord " << statusRecord << endl;

    if (status != statusRecord)
    {

        if (status != 0)
            statusRecord = status;
        msRecord = ros::Time::now();
        // printf("record: %d \n", msRecord);
        // init
        switch (status)
        {
        case 0:
            cout << "middle " << endl;
            break;
        case 1:
            cout << "pitch up init" << endl;
            cout << "take off" << endl;
            ; // arm function

            break;
        case 2:
            cout << "pitch down init" << endl;
            cout << "landing" << endl;

            break;
        case 3:
            cout << "roll up" << endl;

            break;
        case 4:
            cout << "roll down" << endl;
            break;
        case 5:
            cout << "yaw up" << endl;
            break;
        case 6:
            cout << "yaw down" << endl;
            break;
        default:
            break;
        }
    }
    else
    {
        ros::Time ms = ros::Time::now();
        cout<<"start: " << msRecord.toSec() << endl;
        cout <<"current" << ms.toSec() <<endl;

        ros::Duration dt = ms - msRecord;
        switch (statusRecord)
        {
        case 0: // will neven enter
            cout << "Zero" << endl;
            break;
        case 1:
            // take off
            cout << "take off logic" << endl;
            for (int i = 0; i < 3; ++i)
            {
                target_1[i] = target_1_home[i];
                target_2[i] = target_2_home[i];
                target_3[i] = target_3_home[i];
            }
            break;
        case 2:
            // landing
            for (int i = 0; i < 3; ++i)
            {
                target_1[i] = target_1_home[i];
                target_2[i] = target_2_home[i];
                target_3[i] = target_3_home[i];
            }
            break;
        case 5:

            cout << "dt---------------" << dt.toSec()<<endl;
            for (int i = 0; i < 3; ++i)
            {
                target_1[i] = target_1_home[i];
                target_2[i] = target_2_home[i];
                target_3[i] = target_3_home[i];
                if (i == 0)
                {
                    target_1[i] += 1.0 * sin(dt.toSec() );
                    target_2[i] += 1.0 * sin(dt.toSec() );
                    target_3[i] += 1.0 * sin(dt.toSec() );
                }
                if (i == 1)
                {
                    target_1[i] += 2.0 * cos(dt.toSec() );
                    target_2[i] += 2.0 * cos(dt.toSec() );
                    target_3[i] += 2.0 * cos(dt.toSec() );
                }
            }

            break;

        case 6:
            cout << "dt---------------" << dt.toSec()<<endl;
            for (int i = 0; i < 3; ++i)
            {
                target_1[i] = target_1_home[i];
                target_2[i] = target_2_home[i];
                target_3[i] = target_3_home[i];
                if (i == 2)
                {
                    target_1[i] += -1.0 * sin(dt.toSec() /5) + 1;
                    target_2[i] += -1.0 * sin(dt.toSec() /5) + 1;
                    target_3[i] += -1.0 * sin(dt.toSec() /5) + 1;
                }
                if (i == 1)
                {
                    target_1[i] += -1.0 * cos(dt.toSec() /5);
                    target_2[i] += -1.0 * cos(dt.toSec() /5);
                    target_3[i] += -1.0 * cos(dt.toSec() /5);
                }
            }

            break;
        case 4:

            break;
        case 3:

            break;
        default:
            break;
        }
    }

    demo_flight::move_target waypoint_out;

    waypoint_out.header.stamp = ros::Time::now();
    waypoint_out.header.frame_id = "waypoint";

    cout << target_1[0] << target_1[1] << target_1[2] << endl;
    waypoint_out.x = target_1[0];
    waypoint_out.y = target_1[1];
    waypoint_out.z = target_1[2];

    waypoint_out.stick_state = status;

    waypoint_pub.publish(waypoint_out);
}

void vicon_callback(const geometry_msgs::TransformStamped& vicon_data)
{
    position_1[0] = vicon_data.transform.translation.x;
    position_1[1] = vicon_data.transform.translation.y;
    position_1[2] = vicon_data.transform.translation.z;
}
