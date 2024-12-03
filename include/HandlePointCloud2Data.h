#ifndef HANDLE_POINT_CLOUD_DATA_H
#define HANDLE_POINT_CLOUD_DATA_H

//#include <sstream> // For parsing data if needed
//#include <ctime>   // For adding timestamps
//#include <fstream>
//#include <cstring>
//#include <unistd.h>
//#include <arpa/inet.h>
//#include <sys/socket.h>
//#include <iomanip> // For std::put_time

// Add the following includes:
#include <jsoncpp/json/json.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

void handlePointCloud2Data(const char* data, size_t length);

#endif // HANDLE_POINT_CLOUD_DATA_H