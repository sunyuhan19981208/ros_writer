#include "util.h"

bool Util::ReadImage(const char* path, sensor_msgs::Image&image) {
    cv::Mat cv_image = cv::imread(path);
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg(image);
    return true;
}

bool Util::ReadImu(const char* path, sensor_msgs::Imu&image) {
    std::ifstream jfile;
    jfile.open(path);
    nlohmann::json json_data;
    jfile >> json_data;
    image.header = std_msgs::Header();
    image.angular_velocity.x = json_data.at("angular_velocity").at("x_val");
    image.angular_velocity.y = json_data.at("angular_velocity").at("y_val");
    image.angular_velocity.z = json_data.at("angular_velocity").at("z_val");
    image.linear_acceleration.x = json_data.at("linear_acceleration").at("x_val");
    image.linear_acceleration.y = json_data.at("linear_acceleration").at("y_val");
    image.linear_acceleration.z = json_data.at("linear_acceleration").at("z_val");
    image.orientation.x = json_data.at("orientation").at("x_val");
    image.orientation.y = json_data.at("orientation").at("y_val");
    image.orientation.z = json_data.at("orientation").at("z_val");
    image.orientation.w = json_data.at("orientation").at("w_val");
    return true;
}

bool Util::ReadGps(const char* path, sensor_msgs::NavSatFix& gps) {
    std::ifstream jfile;
    jfile.open(path);
    nlohmann::json json_data;
    jfile >> json_data;
    gps.header = std_msgs::Header();
    gps.altitude = json_data.at("gnss").at("geo_point").at("altitude");
    gps.latitude = json_data.at("gnss").at("geo_point").at("latitude");
    gps.longitude = json_data.at("gnss").at("geo_point").at("longitude");
    return true;
}