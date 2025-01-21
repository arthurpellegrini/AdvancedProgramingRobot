#include "ConnectToServer.h"

/**
 * @file ConnectToServer.cpp
 * @brief Implementation of the function to establish a connection to a server on a specific port.
 * 
 * This file contains the implementation of the `ConnectToServer` function, which sets up
 * a TCP connection to a server using the provided IP address and port number.
 */

/**
 * @brief Connects to the server on a specific port.
 * 
 * This function creates a TCP socket, initializes the server address structure,
 * and attempts to connect to the server. If any step fails, the program will
 * terminate with an error message.
 * 
 * @param serverIP The IP address of the server to connect to.
 * @param port The port number to connect to.
 * @return The socket descriptor for the established connection.
 * 
 * @throws Exits the program with an error message if socket creation, server address
 * initialization, or connection fails.
 * 
 * @details The function first creates a socket using `socket()`, then sets up the
 * server address structure with the provided IP address and port number. Finally,
 * it connects to the server using `connect()`. On success, it returns the socket descriptor.
 */
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
