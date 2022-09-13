#include <ros/ros.h>
#include <thread>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "read_msg");
    auto callback = [](const std_msgs::String::ConstPtr &msg){
        ROS_INFO("REPLY: [%s]", msg->data.c_str());
    };
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("str_msg", 100, *callback);
    ros::spin();
}