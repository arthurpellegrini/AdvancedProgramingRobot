#ifndef HANDLE_LASER_SCAN_DATA_H
#define HANDLE_LASER_SCAN_DATA_H

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
#include <sensor_msgs/LaserScan.h>
#include <string>

void handleLaserScanData(const char* data, size_t length);

#endif // HANDLE_LASER_SCAN_DATA_H