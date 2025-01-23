#include <iostream>
#include <thread>
#include <unistd.h>
#include <arpa/inet.h>
#include "ConnectToServer.h"
#include "LaserScanHandler.h"
#include "OdometryHandler.h"
#include "CommanderHandler.h"
#include "SharedMemory.h"
#include "VisualizeDataHandler.h"

/**
 * @file main.cpp
 * @brief Main program to connect to a server and handle LaserScan, Odometry, and Commander data.
 * 
 * This program connects to a server, receives data from multiple ports, and stores the data in shared memory
 * for processing and debugging.
 */

/**
 * @brief Main function of the program.
 * 
 * The program expects the server IP address as a command-line argument and starts threads
 * to handle LaserScan and Odometry data while providing debugging output. It also handles
 * Commander commands via a separate connection.
 * 
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments. The first argument should be the server IP address.
 * @return Returns 0 on successful completion.
 */
int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <Server IP>" << std::endl;
        exit(1);
    }

    const char *serverIP = argv[1];

    /**
     * @brief Semaphore for synchronizing LaserScan data access.
     */
    sem_t* laserSemaphore = nullptr;

    /**
     * @brief Semaphore for synchronizing Odometry data access.
     */
    sem_t* odometrySemaphore = nullptr;

    /**
     * @brief Shared memory structure for storing LaserScan and Odometry data.
     */
    SharedData* sharedMemory = InitializeSharedMemory(laserSemaphore, odometrySemaphore);

    // Connect to the LaserScan port (9997)
    /**
     * @brief Socket for the LaserScan data connection.
     */
    int laserSock = ConnectToServer(serverIP, 9997);

    /**
     * @brief Thread to handle receiving and saving LaserScan data.
     */
    std::thread laserThread([&]() {
        ReceiveAndSaveLaserScanData(laserSock, sharedMemory, laserSemaphore);
    });

    // Connect to the Odometry port (9998)
    /**
     * @brief Socket for the Odometry data connection.
     */
    int odometrySock = ConnectToServer(serverIP, 9998);

    /**
     * @brief Thread to handle receiving and saving Odometry data.
     */
    std::thread odometryThread([&]() {
        ReceiveAndSaveOdometryData(odometrySock, sharedMemory, odometrySemaphore);
    });

    /**
     * @brief Debugging thread to print shared memory data every 5 seconds.
     */
  
 
    // Thread for visualizing data
    std::thread visualizationThread(VisualizeDataHandler);


    // Connect to the Commander port (9999)
    /**
     * @brief Socket for the Commander data connection.
     */
    int commanderSock = ConnectToServer(serverIP, 9999);

    /**
     * @brief Handles Commander commands received from the server.
     * 
     * @param commanderSock Socket for receiving Commander commands.
     */
    CommanderHandler(commanderSock);

    // Wait for data handling threads to complete
    laserThread.join();
    odometryThread.join();
    //debugThread.join();

    // Close the Commander socket
    close(commanderSock);
    std::cout << "[INFO] Commander connection closed." << std::endl;

    // Clean up shared memory and semaphores
    CleanupSharedMemory(sharedMemory, laserSemaphore, odometrySemaphore);

    return 0;
}
