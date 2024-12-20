#include "ConnectToServer.h"

// Function to connect to the server on a specific port
int ConnectToServer(const char *serverIP, unsigned short port) {
    int sock;
    struct sockaddr_in serverAddr;

    // Create socket
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        perror("socket() failed");
        exit(1);
    }

    // Set up the server address structure
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr(serverIP);
    serverAddr.sin_port = htons(port);

    // Connect to the server
    if (connect(sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("connect() failed");
        close(sock);
        exit(1);
    }

    std::cout << "Connected to server on port " << port << std::endl;
    return sock;
}