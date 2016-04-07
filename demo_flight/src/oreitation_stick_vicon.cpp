#include <iostream>
#include <stdio.h>

#include <ros/ros.h>

#include "geometry_msgs/TransformStamped.h"
//#include "TransformStamped.h"
//#include "stick_data.h"
#include "demo_flight/stick_data.h"

#include "math_lib/math_lib.h"

#define RAD2DEG 57.29

using namespace std;
using namespace ros;

ros::Subscriber vicon_sub;
ros::Publisher stick_pub;

vector4f q_r;//r means room
float pitch, roll, yaw;

double roll_dead_min;
double roll_dead_max;

double pitch_dead_min;
double pitch_dead_max;

double yaw_dead_min;
double yaw_dead_max;

double roll_valued_min;
double roll_valued_max;

double pitch_valued_min;
double pitch_valued_max;

double yaw_valued_min;
double yaw_valued_max;

bool roll_valued = false;
bool pitch_valued = false;
bool yaw_valued = false;

void vicon_callback(const geometry_msgs::TransformStamped& vicon_data);

int main(int argc, char** argv)
{
    ros::init(argc,argv,"vicon_stick");
    ros::NodeHandle nh;

    nh.param("roll_dead_min", roll_dead_min, -30.0);
    nh.param("roll_dead_max", roll_dead_max, 30.0);

    nh.param("roll_valued_min", roll_valued_min, -90.0);
    nh.param("roll_valued_max", roll_valued_max, 90.0);

    nh.param("pitch_dead_min", pitch_dead_min, -30.0);
    nh.param("pitch_dead_max", pitch_dead_max, 30.0);

    nh.param("pitch_valued_min", pitch_valued_min, -90.0);
    nh.param("pitch_valued_max", pitch_valued_max, 90.0);

    nh.param("yaw_dead_min", yaw_dead_min, -120.0);
    nh.param("yaw_dead_max", yaw_dead_max, -60.0);

    nh.param("yaw_valued_min", yaw_valued_min, -179.0);
    nh.param("yaw_valued_max", yaw_valued_max, 0.0);


    stick_pub = nh.advertise<demo_flight::stick_data>("stick_data",10);

    vicon_sub = nh.subscribe("/vicon/wand/wand",5,vicon_callback);

    cout<<"start and wait vicon data."<<endl;

    ros::spin();
    return 0;
}

void vicon_callback(const geometry_msgs::TransformStamped& vicon_data)
{

    q_r[0] = vicon_data.transform.rotation.w;
    q_r[1] = vicon_data.transform.rotation.x;
    q_r[2] = vicon_data.transform.rotation.y;
    q_r[3] = vicon_data.transform.rotation.z;

    //cout<<"rotation:"<<" "<<q_r[0]<<" "<<q_r[1]<<" "<<q_r[2]<<" "<<q_r[3]<<endl;

    quat_to_eular(&yaw, &pitch, &roll, q_r);
    yaw *= RAD2DEG;
    pitch *= RAD2DEG;
    roll *= RAD2DEG;
    cout<<"eular:"<<" "<<yaw<<" "<<pitch<<" "<<roll<<endl;

    roll_valued = false;
    pitch_valued = false;
    yaw_valued = false;

    demo_flight::stick_data stick;

    stick.header.stamp =ros::Time::now();
    stick.header.frame_id = "stick";
    stick.valued =0;

    if (yaw > yaw_valued_max)
        yaw_valued = false;
    else if (yaw < yaw_valued_min)
        yaw_valued = false;
    else if (yaw > yaw_dead_min && yaw < yaw_dead_max)
        yaw_valued = false;
    else
        yaw_valued = true;
    cout<<"yaw_valued "<<yaw_valued<<endl;

    if (pitch > pitch_valued_max)
        pitch_valued=false;
    else if (pitch < pitch_valued_min)
        pitch_valued=false;
    else if (pitch > pitch_dead_min && pitch < pitch_dead_max)
        pitch_valued=false;
    else
        pitch_valued=true;
    cout<<"pitch_valued "<<pitch_valued<<endl;

    if (roll > roll_valued_max)
        roll_valued=false;
    else if (roll < roll_valued_min)
        roll_valued=false;
    else if (roll > roll_dead_min && roll < roll_dead_max)
        roll_valued=false;
    else
        roll_valued=true;
    cout<<"roll_valued "<<roll_valued<<endl;


    if(roll_valued==true && roll<roll_dead_min)
    {
        stick.valued =1;
        stick.status =3;
    }
    else if(roll_valued==true && roll>roll_dead_max)
    {
        stick.valued =1;
        stick.status =4;
    }
    else if(pitch_valued==true && pitch>pitch_dead_max)
    {
        stick.valued =1;
        stick.status =1;
    }
    else if(pitch_valued==true && pitch<pitch_dead_min)
    {
        stick.valued =1;
        stick.status =2;
    }
    else if(yaw_valued==true &&yaw>yaw_dead_max)
    {
        stick.valued =1;
        stick.status =5;
    }
    else if(yaw_valued==true &&yaw<yaw_dead_min)
    {
        stick.valued =1;
        stick.status =6;
    }
    int status = stick.status;
    cout<<"status: "<<status<<endl;

    /*if(stick.valued != 0)
    {
        stick.pitch = pitch;
        stick.roll = roll;
        stick.yaw = yaw;
    }*/

    stick_pub.publish(stick);

}
