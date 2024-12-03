#ifndef ROS_TCP_ODOMETRY_LISTENER_H
#define ROS_TCP_ODOMETRY_LISTENER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <cstring>


#define BUFFER_SIZE 65536

void startTCPListener();

#endif // ROS_TCP_ODOMETRY_LISTENER_H
