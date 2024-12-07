#include <iostream>
#include <thread>
#include <cstring>
#include <unistd.h>
#include <atomic>
#include "TalkOnTCPPort.h"
#include "ConnectToServer.h"

std::atomic<bool> running(true);

// Function to send movement commands to the TurtleBot
void startTalkOnTCPPort(int socket, float linear_x, float angular_z) {
    std::string command = "---START---{\"linear\": " + std::to_string(linear_x) +
                          ", \"angular\": " + std::to_string(angular_z) + "}___END___";
    if (send(socket, command.c_str(), command.length(), 0) < 0) {
        perror("send() failed");
        throw std::runtime_error("Failed to send command. Connection may be lost.");
    }
    std::cout << "Command sent: " << command << std::endl;
}

// Function to maintain the connection and handle reconnections
void maintainConnection(const char *serverIP, int port, int &tcpSock) {
    while (running) {
        if (tcpSock < 0) {
            std::cerr << "Connection lost. Reconnecting..." << std::endl;
            sleep(2); // Wait before retrying
            tcpSock = ConnectToServer(serverIP, port);
            if (tcpSock >= 0) {
                std::cout << "Reconnected to server." << std::endl;
                try {
                    startTalkOnTCPPort(tcpSock, 0.0, 0.0); // Send stop command after reconnecting
                } catch (const std::exception &e) {
                    std::cerr << "Error after reconnect: " << e.what() << std::endl;
                }
            }
        } else {
            sleep(1); // Check periodically
        }
    }
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <IP> <PORT>\n";
        return 1;
    }

    const char *serverIP = argv[1];
    int port = std::stoi(argv[2]);

    int tcpSock = ConnectToServer(serverIP, port);
    if (tcpSock < 0) {
        std::cerr << "Failed to connect to server." << std::endl;
        return 1;
    }

    std::thread connectionThread([&]() {
        maintainConnection(serverIP, port, tcpSock);
    });

    std::cout << "Connected to TurtleBot. Enter movement commands:" << std::endl;
    std::cout << "Format: <linear_x> <angular_z> (e.g., 0.5 0.2)" << std::endl;
    std::cout << "Enter 'q' to quit." << std::endl;

    while (running) {
        std::string input;
        std::cout << "> ";
        std::getline(std::cin, input);

        if (input == "q" || input == "Q") {
            std::cout << "Exiting..." << std::endl;
            running = false;
            break;
        }

        float linear_x, angular_z;
        try {
            size_t space_pos = input.find(' ');
            if (space_pos == std::string::npos) {
                throw std::invalid_argument("Invalid format. Use <linear_x> <angular_z>.");
            }

            linear_x = std::stof(input.substr(0, space_pos));
            angular_z = std::stof(input.substr(space_pos + 1));

            startTalkOnTCPPort(tcpSock, linear_x, angular_z);
        } catch (const std::exception &e) {
            std::cerr << "Error: " << e.what() << std::endl;
            std::cerr << "Please enter the command in the correct format: <linear_x> <angular_z>" << std::endl;
        }
    }

    startTalkOnTCPPort(tcpSock, 0.0, 0.0); // Send stop command before exit
    running = false;
    connectionThread.join();

    if (tcpSock >= 0) {
        close(tcpSock);
        std::cout << "Connection closed." << std::endl;
    }

    return 0;
}
