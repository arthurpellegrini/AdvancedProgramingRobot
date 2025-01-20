#pragma once
#include <semaphore.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

/**
 * @file SharedMemory.h
 * @brief Provides shared memory and semaphore management for data synchronization.
 * 
 * This header defines the structure and functions used for initializing, managing,
 * and cleaning up shared memory and semaphores for LaserScan and Odometry data.
 */

/**
 * @brief Structure to store shared data for LaserScan and Odometry.
 * 
 * This structure holds buffers for storing LaserScan and Odometry data
 * received from the server. It is designed for use in a multi-threaded
 * environment with semaphore synchronization.
 */
struct SharedData {
    char laserScanData[1024]; ///< Buffer for LaserScan data.
    char odometryData[1024]; ///< Buffer for Odometry data.
};

/**
 * @brief Initializes shared memory and semaphores.
 * 
 * This function allocates memory for the shared data structure and initializes
 * semaphores for synchronizing access to LaserScan and Odometry data.
 * 
 * @param laserSemaphore Reference to a pointer for the LaserScan semaphore.
 * @param odometrySemaphore Reference to a pointer for the Odometry semaphore.
 * @return Pointer to the allocated and initialized shared data structure.
 * 
 * @note The returned shared memory must be cleaned up with `CleanupSharedMemory` 
 * to prevent memory leaks and ensure semaphore destruction.
 */
SharedData* InitializeSharedMemory(sem_t*& laserSemaphore, sem_t*& odometrySemaphore);

/**
 * @brief Cleans up shared memory and semaphores.
 * 
 * This function releases memory allocated for the shared data structure and
 * destroys the semaphores used for synchronization.
 * 
 * @param shared Pointer to the shared data structure to be freed.
 * @param laserSemaphore Pointer to the LaserScan semaphore to be destroyed.
 * @param odometrySemaphore Pointer to the Odometry semaphore to be destroyed.
 */
void CleanupSharedMemory(SharedData* shared, sem_t* laserSemaphore, sem_t* odometrySemaphore);
