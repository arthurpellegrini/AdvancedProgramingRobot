#ifndef LASER_SCAN_HANDLER_H
#define LASER_SCAN_HANDLER_H

#include <sstream> // For parsing data if needed
#include <ctime>   // For adding timestamps
#include <iostream>
#include <fstream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iomanip> // For std::put_time

// Buffer size for receiving data
#define BUFFER_SIZE 1024

// Function to receive and save LaserScan data
void ReceiveAndSaveLaserScanData(int sock);

#endif // LASER_SCAN_HANDLER_H
