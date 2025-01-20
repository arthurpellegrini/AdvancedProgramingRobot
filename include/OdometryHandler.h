#ifndef ODOMETRY_HANDLER_H
#define ODOMETRY_HANDLER_H

#include <sstream> // For parsing data if needed
#include <ctime>   // For adding timestamps
#include <iostream>
#include <fstream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iomanip> // For std::put_time
#include "SharedMemory.h"
#include <semaphore.h>

/**
 * @file OdometryHandler.h
 * @brief Handles the reception and storage of Odometry data.
 * 
 * This header defines functions and constants used for processing
 * Odometry data received from a server. The data is synchronized
 * using semaphores and stored in shared memory for further use.
 */

/**
 * @brief Defines the buffer size for receiving Odometry data.
 * 
 * This constant is used to allocate memory for receiving data packets
 * transmitted from the server.
 */
#define BUFFER_SIZE 1024

/**
 * @brief Receives and saves Odometry data from a socket.
 * 
 * This function continuously receives Odometry data from the given socket,
 * processes it, and stores it in shared memory. Synchronization is ensured
 * through the provided semaphore, allowing safe concurrent access to shared
 * resources.
 * 
 * @param sock The socket descriptor used to receive Odometry data.
 * @param shared Pointer to the shared memory structure where Odometry data will be stored.
 * @param odometrySemaphore Pointer to the semaphore used for synchronizing access to shared memory.
 * 
 * @note Ensure that the socket is connected to the correct server and port before calling this function.
 * 
 * @details The function operates in a loop, reading incoming data from the server,
 * adding timestamps, and writing the data to shared memory. It is designed to run
 * as part of a multi-threaded system where thread safety is managed via semaphores.
 */
void ReceiveAndSaveOdometryData(int sock, SharedData* shared, sem_t* odometrySemaphore);

#endif // ODOMETRY_HANDLER_H
