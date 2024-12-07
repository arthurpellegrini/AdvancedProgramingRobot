#include <thread>
#include "ListenOnTCPPort.h"
#include "ConnectToServer.h"

#define BUFFER_SIZE 1024

void startlistenOnTCPPort(int sock) {
    char buffer[BUFFER_SIZE];
    ssize_t bytesReceived;

    while ((bytesReceived = recv(sock, buffer, BUFFER_SIZE - 1, 0)) > 0) {
        buffer[bytesReceived] = '\0'; // Null-terminate the received data

        // Remove Trailing Newline-Symb
        while (bytesReceived > 0 && (buffer[bytesReceived - 1] == '\n' || buffer[bytesReceived - 1] == '\r')) {
            buffer[--bytesReceived] = '\0';
        }

        // create timestamp 
        std::time_t now = std::time(nullptr);
        std::tm *localTime = std::localtime(&now); // convert to local time

        // print timestamp and data
        std::cout << "[" << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << "] " << buffer << std::endl;
    }

    if (bytesReceived < 0) {
        perror("[ERROR] recv() failed");
        exit(1);
    } else {
        std::cout << "[DEBUG] Connection closed by client." << std::endl;
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <IP> <PORT>\n";
        return 1;
    }

    const char *serverIP = argv[1];
    int port = std::stoi(argv[2]);

    int tcpSock = ConnectToServer(serverIP, port);
    std::thread tcpThread(startlistenOnTCPPort, tcpSock);

    // Wait for the thread to complete
    tcpThread.join();

    return 0;

}