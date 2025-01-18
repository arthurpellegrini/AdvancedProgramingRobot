
#pragma once
#include <semaphore.h>

struct SharedData {
    char laserScanData[1024];
    char odometryData[1024];
};

SharedData* InitializeSharedMemory(sem_t*& laserSemaphore, sem_t*& odometrySemaphore);
void CleanupSharedMemory(SharedData* shared, sem_t* laserSemaphore, sem_t* odometrySemaphore);