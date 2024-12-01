#include "SendSimulatedData.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstdlib>

// Function to simulate continuous data sending
void SendSimulatedData(int clientSock, const std::string &dataType) {
    std::string data;
    while (true) {
        if (dataType == "LaserScan") {
            data = "10:2.5\n"; // e.g.: angle:distance
        } else if (dataType == "Odometry") {
            data = "1.0:2.0:0.5\n"; // e.g.: x:y:theta
        } else {
            data = "Unknown,Data,Format\n";
        }

        if (send(clientSock, data.c_str(), data.size(), 0) != (ssize_t)data.size()) {
            perror("send() failed");
            break;
        }
        std::cout << "Sent: " << data;
        sleep(2); // Wait 2 second before sending more data
    }
    close(clientSock);
}