#include "ChattyTCPServer.h"
#include "SendSimulatedData.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstdlib>

// Function to start the chatty TCP server
void StartChattyTCPServer(unsigned short port) {
    int serverSock, clientSock;
    struct sockaddr_in serverAddr, clientAddr;
    unsigned int clientLen;

    // Create the socket
    if ((serverSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        perror("socket() failed");
        exit(1);
    }

    // Set up the server address
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(port);

    // Bind the socket to the port
    if (bind(serverSock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("bind() failed");
        close(serverSock);
        exit(1);
    }

    // Listen for incoming connections
    if (listen(serverSock, 5) < 0) {
        perror("listen() failed");
        close(serverSock);
        exit(1);
    }

    std::cout << "Chatting on port " << port << std::endl;

    // Handle clients in a loop
    for (;;) {
        clientLen = sizeof(clientAddr);
        if ((clientSock = accept(serverSock, (struct sockaddr *)&clientAddr, &clientLen)) < 0) {
            perror("accept() failed");
            continue;
        }

        std::cout << "Client connected: " << inet_ntoa(clientAddr.sin_addr) << std::endl;

        // Send simulated data based on the port
        if (port == 9997) {
            SendSimulatedData(clientSock, "LaserScan");
        } else if (port == 9998) {
            SendSimulatedData(clientSock, "Odometry");
        }
    }

    close(serverSock);
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <Port>" << std::endl;
        return 1;
    }

    unsigned short port = std::atoi(argv[1]);
    StartChattyTCPServer(port);

    return 0;
}
