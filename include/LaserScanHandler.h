#ifndef LASER_SCAN_HANDLER_H
#define LASER_SCAN_HANDLER_H

#include <sstream> // For parsing data if needed
#include <ctime>   // For adding timestamps
#include <jsoncpp/json/json.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iomanip> // For std::put_time
#include <cstdlib> // For system()
#include "SharedMemory.h"
#include <semaphore.h>









/**
 * @file LaserScanHandler.h
 * @brief Handles the reception and storage of LaserScan data.
 * 
 * This header defines functions and constants used for processing
 * LaserScan data received from a server. The data is synchronized
 * using semaphores and stored in shared memory for further processing.
 */

/**
 * @brief Defines the buffer size for receiving LaserScan data.
 * 
 * This constant is used to allocate memory for receiving data packets.
 */
#define BUFFER_SIZE 1024

/**
 * @brief Receives and saves LaserScan data from a socket.
 * 
 * This function handles the reception of LaserScan data from the given
 * socket, processes it, and stores it in shared memory. Synchronization
 * is managed using the provided semaphore to ensure thread safety.
 * 
 * @param sock The socket descriptor used to receive data.
 * @param shared Pointer to the shared memory structure where data will be stored.
 * @param laserSemaphore Pointer to the semaphore used for synchronizing access to shared memory.
 * 
 * @note Ensure the socket is connected to the correct server and port before calling this function.
 * 
 * @details The function operates in a loop, continuously receiving data
 * and writing it to the shared memory. Each data entry is timestamped
 * for debugging and analysis purposes.
 */
void ReceiveAndSaveLaserScanData(int sock, SharedData* shared, sem_t* laserSemaphore);

#endif // LASER_SCAN_HANDLER_H
