#include <iostream>
#include <thread>
#include <unistd.h>
#include <arpa/inet.h>
#include "ConnectToServer.h"
#include "LaserScanHandler.h"
#include "OdometryHandler.h"
#include "CommanderHandler.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <Server IP>" << std::endl;
        exit(1);
    }

    const char *serverIP = argv[1];

    // Connect to the LaserScan port (9997)
    int laserSock = ConnectToServer(serverIP, 9997);
    std::thread laserThread(ReceiveAndSaveLaserScanData, laserSock);

    // Connect to the Odometry port (9998)
    int odometrySock = ConnectToServer(serverIP, 9998);
    std::thread odometryThread(ReceiveAndSaveOdometryData, odometrySock);

    // Connect to the Commander port (9999)
    int commanderSock = ConnectToServer(serverIP, 9999);
    CommanderHandler(commanderSock);

    // Wait for data handling threads to complete
    laserThread.join();
    odometryThread.join();

    // Close the Commander socket
    close(commanderSock);
    std::cout << "[INFO] Commander connection closed." << std::endl;

    return 0;
}
