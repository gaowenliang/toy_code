#include <iostream>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image, const ImuConstPtr& imu)
{
    ROS_INFO("GET MSG");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;

    message_filters::Subscriber<Image> image_sub(nh, "/camera/image_raw", 1);
    message_filters::Subscriber<Imu> imu_sub(nh, "/imu_3dm_gx4/imu", 1);

    typedef message_filters::sync_policies::ApproximateTime<Image,Imu> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(20), image_sub, imu_sub);

    sync.registerCallback(boost::bind(&callback, _1,_2));

    ros::spin();

    return 0;
}
