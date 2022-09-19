#include <ros/ros.h>
#include <thread>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <experimental/filesystem>
#include "util.h"

enum class AirsimToRos{
    IMG1,
    IMG2,
    IMU,
    GPS
};

void ReadDataFromFile(std::vector<std::pair<long long, AirsimToRos>> & vec){
    std::string img_path = "/home/sunyuhan/syhdata3/image3";
    std::string imu_path = "/home/sunyuhan/syhdata3/imu";
    std::string gps_path = "/home/sunyuhan/syhdata3/gps";
    try{
        for(auto it: std::experimental::filesystem::directory_iterator(img_path)){
            std::string path = it.path();
            auto pos1 = path.find_last_of('/');
            auto pos2 = path.find_last_of('_');
            std::string timestamp(path.begin() + pos1 +1, path.begin() + pos2);
            auto tll = strtoll(timestamp.c_str(), NULL, 10);
            int num = *(path.begin() + pos2 +1) - '0';
            AirsimToRos suffix = num == 1 ? AirsimToRos::IMG1 : AirsimToRos::IMG2;
            vec.push_back({tll, suffix});
        }
        for(auto it: std::experimental::filesystem::directory_iterator(imu_path)){
            std::string path = it.path();
            auto pos1 = path.find_last_of('/');
            auto pos2 = path.find_last_of('.');
            std::string timestamp(path.begin() + pos1 +1, path.begin() + pos2);
            auto tll = strtoll(timestamp.c_str(), NULL, 10);
            vec.push_back({tll, AirsimToRos::IMU});
        }
        for(auto it: std::experimental::filesystem::directory_iterator(gps_path)){
            std::string path = it.path();
            auto pos1 = path.find_last_of('/');
            auto pos2 = path.find_last_of('.');
            std::string timestamp(path.begin() + pos1 +1, path.begin() + pos2);
            auto tll = strtoll(timestamp.c_str(), NULL, 10);
            vec.push_back({tll, AirsimToRos::GPS});
        }
    } catch(const std::exception& e){
        std::cout << e.what() << std::endl;
    }
    std::sort(vec.begin(), vec.end(), [](const auto& it1, const auto& it2){
        return it1.first < it2.first;
    });
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "write_msg_to_ros");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("str_msg", 100);
    ros::Publisher img_pub1 = n.advertise<sensor_msgs::Image>("test/image1", 100);
    ros::Publisher img_pub2 = n.advertise<sensor_msgs::Image>("test/image2", 100);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("test/imu", 100);
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("test/gps", 100);
    ros::Rate loop_rate(100);

    auto pub_img1 = [&img_pub1](long long ts)
    {
        sensor_msgs::Image img;
        std::stringstream img_path;
        img_path << "/home/sunyuhan/syhdata3/image3/" << ts << "_1.png";
        Util::ReadImage(img_path.str().c_str(), img);
        img_pub1.publish(img);
    };

    auto pub_img2 = [&img_pub2](long long ts)
    {
        sensor_msgs::Image img;
        std::stringstream img_path;
        img_path << "/home/sunyuhan/syhdata3/image3/" << ts << "_2.png";
        Util::ReadImage(img_path.str().c_str(), img);
        img_pub2.publish(img);
    };

    auto pub_imu = [&imu_pub](long long ts)
    {
        sensor_msgs::Imu imu;
        std::stringstream imu_path;
        imu_path << "/home/sunyuhan/syhdata3/imu/" << ts << ".bin";
        Util::ReadImu(imu_path.str().c_str(), imu);
        imu_pub.publish(imu);
    };

    auto pub_gps = [&gps_pub](long long ts)
    {
        sensor_msgs::NavSatFix gps;
        std::stringstream gps_path;
        gps_path << "/home/sunyuhan/syhdata3/gps/" << ts << ".bin";
        Util::ReadGps(gps_path.str().c_str(), gps);
        gps_pub.publish(gps);
    };
    std::vector<std::pair<long long, AirsimToRos>> vec;
    ReadDataFromFile(vec);

    for(const auto &[ts, suffix] : vec)
    {
        std::cout << ts <<" " << static_cast<int>(suffix)<< std::endl;
        if(!ros::ok())
            break;
        switch (suffix)
        {
        case AirsimToRos::IMG1:
            pub_img1(ts);
            break;
        case AirsimToRos::IMG2:
            pub_img2(ts);
            break;
        case AirsimToRos::IMU:
            pub_imu(ts);
            break;
        case AirsimToRos::GPS:
            pub_gps(ts);
            break;

        default:
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}