#ifndef HANDLE_ODOMETRY_DATA_H
#define HANDLE_ODOMETRY_DATA_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <jsoncpp/json/json.h>
#include <string>
#include <sstream>


void handleOdometryData(const char* data, size_t length);

#endif // HANDLE_ODOMETRY_DATA_H
