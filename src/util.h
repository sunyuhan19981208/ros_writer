#ifndef UTIL_H_
#define UTIL_H_

#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
class Util {
public:
    /**
     * @brief read image from file and write to ros
     *
     * @param path
     * @return true
     * @return false
     */
    static bool ReadImage(const char *path, sensor_msgs::Image &);

    /**
     * @brief read imu from file and write to ros
     *
     * @param path
     * @return true
     * @return false
     */
    static bool ReadImu(const char *path, sensor_msgs::Imu &);

    /**
     * @brief read gps from file and write to ros
     *
     * @param path
     * @return true
     * @return false
     */
    static bool ReadGps(const char *path, sensor_msgs::NavSatFix &);
};

#endif