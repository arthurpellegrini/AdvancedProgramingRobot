#include <iostream>
#include <thread>
#include <unistd.h>
#include <arpa/inet.h>
#include "ConnectToServer.h"
#include "LaserScanHandler.h"
#include "OdometryHandler.h"
#include "CommanderHandler.h"
#include "SharedMemory.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <Server IP>" << std::endl;
        exit(1);
    }

    const char *serverIP = argv[1];

    sem_t* laserSemaphore = nullptr;
    sem_t* odometrySemaphore = nullptr;
    SharedData* sharedMemory = InitializeSharedMemory(laserSemaphore, odometrySemaphore);

    // Connect to the LaserScan port (9997)
    int laserSock = ConnectToServer(serverIP, 9997);
    std::thread laserThread([&]() {
        ReceiveAndSaveLaserScanData(laserSock, sharedMemory, laserSemaphore);
    });

    // Connect to the Odometry port (9998)
    int odometrySock = ConnectToServer(serverIP, 9998);
    std::thread odometryThread([&]() {
        ReceiveAndSaveOdometryData(odometrySock, sharedMemory, odometrySemaphore);
    });

    // Debug thread to print shared memory data every 5 seconds
    std::thread debugThread([&]() {
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(5));

            sem_wait(laserSemaphore);
            std::cout << "[DEBUG] LaserScan Data: " << sharedMemory->laserScanData << std::endl;
            sem_post(laserSemaphore);

            sem_wait(odometrySemaphore);
            std::cout << "[DEBUG] Odometry Data: " << sharedMemory->odometryData << std::endl;
            sem_post(odometrySemaphore);
        }
    });

    // Connect to the Commander port (9999)
    int commanderSock = ConnectToServer(serverIP, 9999);
    CommanderHandler(commanderSock);

    // Wait for data handling threads to complete
    laserThread.join();
    odometryThread.join();
    debugThread.join(); 

    // Close the Commander socket
    close(commanderSock);
    std::cout << "[INFO] Commander connection closed." << std::endl;

    CleanupSharedMemory(sharedMemory, laserSemaphore, odometrySemaphore);

    return 0;
}
