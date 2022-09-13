#include <ros/ros.h>
#include <thread>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include "util.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "write_msg_to_ros");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("str_msg", 100);
    ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("test/image1", 100);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("test/imu", 100);
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("test/gps", 100);
    ros::Rate loop_rate(10);
    int i = 0;

    auto pub_img = [&img_pub](int i){
        sensor_msgs::Image img;
        std::stringstream img_path;
        img_path << "/home/sunyuhan/syhdata/image3/" << i << "_1.png";
        Util::ReadImage(img_path.str().c_str(), img);
        img_pub.publish(img);
    };

    auto pub_imu = [&imu_pub](int i){
        sensor_msgs::Imu imu;
        std::stringstream imu_path;
        imu_path << "/home/sunyuhan/syhdata/imu/" << i << ".bin";
        // TODO : extend to 4 topics
        Util::ReadImu(imu_path.str().c_str(), imu);
        imu_pub.publish(imu);
    };

    auto pub_gps = [&gps_pub](int i){
        sensor_msgs::NavSatFix gps;
        std::stringstream gps_path;
        gps_path << "/home/sunyuhan/syhdata/gps/" << i << ".bin";
        Util::ReadGps(gps_path.str().c_str(), gps);
        gps_pub.publish(gps);
    };

    while(ros::ok() && i<300){
        std_msgs::String msg;
        msg.data = "test";
        pub.publish(msg);
        pub_img(i);
        ros::spinOnce();
        loop_rate.sleep();
    }
}