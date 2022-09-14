#include "util.h"

void TestReadImage() {
    std::stringstream path;
    sensor_msgs::Image img;
    path<<"/home/sunyuhan/syhdata/image3/" << 13 << "_1.png";
    Util::ReadImage(path.str().c_str(), img);
}

void TestReadImu() {
    std::stringstream path;
    sensor_msgs::Imu imu;
    path<<"/home/sunyuhan/syhdata/imu/" << 1 << ".bin";
    Util::ReadImu(path.str().c_str(), imu);
}

void TestReadGps() {
    std::stringstream path;
    sensor_msgs::NavSatFix gps;
    path<<"/home/sunyuhan/syhdata/gps/" << 1 << ".bin";
    Util::ReadGps(path.str().c_str(), gps);
}

int main() {
    TestReadImage();
    TestReadImu();
    TestReadGps();
}