#include <thread>
#include "ListenOnTCPPort.h"
#include "ConnectToServer.h"

#define BUFFER_SIZE 1024 ///< Buffer size for TCP communication

/**
 * @file ListenOnTCPPort.cpp
 * @brief Implementation of TCP listening and data processing.
 * 
 * This file contains the implementation of the `startlistenOnTCPPort` function
 * for receiving and printing data from a TCP connection, along with a simple
 * `main` function to demonstrate its usage.
 */

/**
 * @brief Listens on a TCP socket and processes incoming data.
 * 
 * This function continuously receives data from the specified socket and prints
 * it to the console with a timestamp. It also removes trailing newline characters
 * for clean output.
 * 
 * @param sock The socket descriptor for the TCP connection.
 * 
 * @details The function reads data from the socket in a loop, processing each
 * received message. The data is timestamped with the current local time and
 * displayed on the console. If an error occurs during `recv()`, the function
 * terminates the program.
 * 
 * @note Ensure that the socket is properly connected before calling this function.
 * The function is designed to run in a separate thread for non-blocking operation.
 */
void startlistenOnTCPPort(int sock) {
    char buffer[BUFFER_SIZE];
    ssize_t bytesReceived;

    while ((bytesReceived = recv(sock, buffer, BUFFER_SIZE - 1, 0)) > 0) {
        buffer[bytesReceived] = '\0'; // Null-terminate the received data

        // Remove trailing newline symbols
        while (bytesReceived > 0 && (buffer[bytesReceived - 1] == '\n' || buffer[bytesReceived - 1] == '\r')) {
            buffer[--bytesReceived] = '\0';
        }

        // Create timestamp
        std::time_t now = std::time(nullptr);
        std::tm *localTime = std::localtime(&now); // Convert to local time

        // Print timestamp and data
        std::cout << "[" << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << "] " << buffer << std::endl;
    }

    // Handle errors or connection closure
    if (bytesReceived < 0) {
        perror("[ERROR] recv() failed");
        exit(1);
    } else {
        std::cout << "[DEBUG] Connection closed by client." << std::endl;
    }
}

/**
 * @brief Main function to demonstrate TCP listening.
 * 
 * This program connects to a TCP server using the provided IP address and port,
 * starts a thread to listen for incoming data, and prints the received data
 * with timestamps.
 * 
 * @param argc The number of command-line arguments.
 * @param argv The command-line arguments. Expects the server IP and port as arguments.
 * @return Returns 0 on successful execution, or 1 on invalid usage.
 * 
 * @usage Example: `./ListenOnTCPPort <IP> <PORT>`
 * 
 * @details The `main` function establishes a connection to the server using
 * `ConnectToServer` and starts a thread to run the `startlistenOnTCPPort` function.
 * The program waits for the thread to complete before exiting.
 */
int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <IP> <PORT>\n";
        return 1;
    }

    const char *serverIP = argv[1];
    int port = std::stoi(argv[2]);

    // Connect to the server
    int tcpSock = ConnectToServer(serverIP, port);

    // Start listening on the TCP port in a separate thread
    std::thread tcpThread(startlistenOnTCPPort, tcpSock);

    // Wait for the thread to complete
    tcpThread.join();

    return 0;
}
