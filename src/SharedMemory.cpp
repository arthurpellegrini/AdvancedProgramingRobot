
#include "SharedMemory.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#define LASER_SEM_NAME "/laserSem"
#define ODOM_SEM_NAME  "/odomSem"

SharedData* InitializeSharedMemory(sem_t*& laserSemaphore, sem_t*& odometrySemaphore) {
    int fd = shm_open("/shared_data", O_CREAT | O_RDWR, 0666);
    ftruncate(fd, sizeof(SharedData));
    void* addr = mmap(nullptr, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    SharedData* shared = static_cast<SharedData*>(addr);
    std::memset(shared, 0, sizeof(SharedData));

    laserSemaphore = sem_open(LASER_SEM_NAME, O_CREAT, 0666, 1);
    odometrySemaphore = sem_open(ODOM_SEM_NAME, O_CREAT, 0666, 1);

    return shared;
}

void CleanupSharedMemory(SharedData* shared, sem_t* laserSemaphore, sem_t* odometrySemaphore) {
    munmap(shared, sizeof(SharedData));
    shm_unlink("/shared_data");

    sem_close(laserSemaphore);
    sem_close(odometrySemaphore);

    sem_unlink(LASER_SEM_NAME);
    sem_unlink(ODOM_SEM_NAME);
}