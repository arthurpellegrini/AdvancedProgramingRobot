#include <iostream>
#include <thread>
#include <cstring>
#include <unistd.h>
#include "TalkOnTCPPort.h"
#include "ConnectToServer.h"

// Function to send movement commands to the TurtleBot
void startTalkOnTCPPort(int socket, float linear_x, float angular_z) {
    // Create a JSON-like string for the movement command
    std::string command = "{\"linear_x\": " + std::to_string(linear_x) +
                          ", \"angular_z\": " + std::to_string(angular_z) + "}";

    // Send the command to the server
    if (send(socket, command.c_str(), command.length(), 0) < 0) {
        perror("send() failed");
        close(socket);
        exit(1);
    }

    std::cout << "Command sent: " << command << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <IP> <PORT>\n";
        return 1;
    }

    const char *serverIP = argv[1];
    int port = std::stoi(argv[2]);

    int tcpSock = ConnectToServer(serverIP, port);
    std::thread tcpThread([&]() {
        startTalkOnTCPPort(tcpSock, 0.0, 0.0);
    });

    std::cout << "Connected to TurtleBot. Enter movement commands:" << std::endl;
    std::cout << "Format: <linear_x> <angular_z> (e.g., 0.5 0.2)" << std::endl;
    std::cout << "Enter 'q' to quit." << std::endl;

    while (true) {
        std::string input;
        std::cout << "> ";
        std::getline(std::cin, input);

        // Check for quit command
        if (input == "q" || input == "Q") {
            std::cout << "Exiting..." << std::endl;
            break;
        }

        // Parse the input for linear_x and angular_z
        float linear_x, angular_z;
        try {
            size_t space_pos = input.find(' ');
            if (space_pos == std::string::npos) {
                throw std::invalid_argument("Invalid format. Use <linear_x> <angular_z>.");
            }

            linear_x = std::stof(input.substr(0, space_pos));
            angular_z = std::stof(input.substr(space_pos + 1));

            // Send the movement command
            startTalkOnTCPPort(tcpSock, linear_x, angular_z);
        } catch (const std::exception &e) {
            std::cerr << "Error: " << e.what() << std::endl;
            std::cerr << "Please enter the command in the correct format: <linear_x> <angular_z>" << std::endl;
        }
    }

    // Stop the TurtleBot before exiting
    startTalkOnTCPPort(tcpSock, 0.0, 0.0);

    // Close the connection
    close(tcpSock);
    std::cout << "Connection closed." << std::endl;

    return 0;
}
