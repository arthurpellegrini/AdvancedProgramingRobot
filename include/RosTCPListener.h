// Header File: ros_tcp_listener.h

#ifndef ROS_TCP_LISTENER_H
#define ROS_TCP_LISTENER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <jsoncpp/json/json.h>

#define PORT 9997
#define BUFFER_SIZE 65536

void handlePointCloud2Data(const char* data, size_t length);
void handleLaserScanData(const char* data, size_t length);
void startTCPListener();

#endif // ROS_TCP_LISTENER_H
