#include "SharedMemory.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#define LASER_SEM_NAME "/laserSem" ///< Name for the semaphore used to synchronize LaserScan data
#define ODOM_SEM_NAME  "/odomSem" ///< Name for the semaphore used to synchronize Odometry data

/**
 * @file SharedMemory.cpp
 * 
 * @brief Initializes shared memory and semaphores for inter-process communication.
 * 
 * This function sets up shared memory and semaphores for synchronizing access
 * to shared LaserScan and Odometry data between processes. It creates or opens
 * the shared memory and semaphores, initializing them as needed.
 * 
 * @param laserSemaphore Reference to a pointer for the LaserScan semaphore.
 * @param odometrySemaphore Reference to a pointer for the Odometry semaphore.
 * @return Pointer to the allocated and initialized shared memory structure.
 * 
 * @details The function uses `shm_open` to create or open shared memory and maps
 * it into the process address space. Semaphores are created or opened with initial
 * values of `1` to allow for mutual exclusion during data access.
 * 
 * @note The shared memory and semaphores must be cleaned up using `CleanupSharedMemory`
 * to prevent resource leaks.
 */
SharedData* InitializeSharedMemory(sem_t*& laserSemaphore, sem_t*& odometrySemaphore) {
    int fd = shm_open("/shared_data", O_CREAT | O_RDWR, 0666); // Create or open shared memory
    ftruncate(fd, sizeof(SharedData)); // Set the size of shared memory
    void* addr = mmap(nullptr, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0); // Map to address space
    close(fd);

    SharedData* shared = static_cast<SharedData*>(addr); // Cast to SharedData pointer
    std::memset(shared, 0, sizeof(SharedData)); // Initialize shared memory to zero

    laserSemaphore = sem_open(LASER_SEM_NAME, O_CREAT, 0666, 1); // Create or open LaserScan semaphore
    odometrySemaphore = sem_open(ODOM_SEM_NAME, O_CREAT, 0666, 1); // Create or open Odometry semaphore

    return shared;
}

/**
 * @brief Cleans up shared memory and semaphores.
 * 
 * This function unmaps and unlinks the shared memory and closes and unlinks
 * the semaphores created or used during the program execution.
 * 
 * @param shared Pointer to the shared memory structure to be cleaned up.
 * @param laserSemaphore Pointer to the semaphore for LaserScan data synchronization.
 * @param odometrySemaphore Pointer to the semaphore for Odometry data synchronization.
 * 
 * @details The function ensures that all resources allocated for shared memory
 * and semaphores are released properly. This includes unmapping the memory,
 * unlinking the shared memory file, and closing and unlinking the semaphores.
 */
void CleanupSharedMemory(SharedData* shared, sem_t* laserSemaphore, sem_t* odometrySemaphore) {
    munmap(shared, sizeof(SharedData)); // Unmap shared memory
    shm_unlink("/shared_data"); // Unlink shared memory

    sem_close(laserSemaphore); // Close LaserScan semaphore
    sem_close(odometrySemaphore); // Close Odometry semaphore

    sem_unlink(LASER_SEM_NAME); // Unlink LaserScan semaphore
    sem_unlink(ODOM_SEM_NAME); // Unlink Odometry semaphore
}
