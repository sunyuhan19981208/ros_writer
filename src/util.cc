#include "util.h"

bool Util::ReadImage(const char* path, sensor_msgs::Image&image) {
    cv::Mat cv_image = cv::imread(path);
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg(image);
    return true;
}

bool Util::ReadImu(const char* path, sensor_msgs::Imu&image) {
    // TODO
}

bool Util::ReadGps(const char* path, sensor_msgs::NavSatFix& gps) {
    // TODO
}