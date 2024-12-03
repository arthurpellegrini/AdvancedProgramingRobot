#ifndef ROS_TCP_LASER_LISTENER_H
#define ROS_TCP_LASER_LISTENER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <cstring>

#define PORT 9997
#define BUFFER_SIZE 65536

void startTCPListener();

#endif // ROS_TCP_LASER_LISTENER_H
