#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <string>
#include <sstream>

#define BUFFER_SIZE 1024

// Function to connect to the server
int ConnectToCommanderServer(const char *serverIP, int port) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        exit(1);
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    if (inet_pton(AF_INET, serverIP, &serverAddr.sin_addr) <= 0) {
        perror("Invalid address or address not supported");
        exit(1);
    }

    if (connect(sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("Connection failed");
        exit(1);
    }

    std::cout << "[INFO] Connected to Commander Server at " << serverIP << ":" << port << std::endl;
    return sock;
}

// Function to send movement commands
void SendMovementCommand(int sock, float linear, float angular) {
    std::ostringstream oss;
    oss << "---START---{";
    oss << "\"linear\": " << linear << ", ";
    oss << "\"angular\": " << angular;
    oss << "}___END___";
    std::string command = oss.str();

    if (send(sock, command.c_str(), command.length(), 0) < 0) {
        perror("[ERROR] Failed to send command");
    } else {
        std::cout << "[DEBUG] Sent command: " << command << std::endl;
    }
}

// Movement functions
void MoveForward(int sock) {
    SendMovementCommand(sock, 0.1, 0.0);
}

void MoveBackward(int sock) {
    SendMovementCommand(sock, -0.1, 0.0);
}

void MoveLeft(int sock) {
    SendMovementCommand(sock, 0.0, 0.5);
}

void MoveRight(int sock) {
    SendMovementCommand(sock, 0.0, -0.5);
}

void StopRobot(int sock) {
    SendMovementCommand(sock, 0.0, 0.0);
}

void RotateLeft90(int sock) {
    SendMovementCommand(sock, 0.0, 1.57); // Approx. 90 degrees in radians
}

void RotateRight90(int sock) {
    SendMovementCommand(sock, 0.0, -1.57); // Approx. -90 degrees in radians
}

void Rotate180(int sock) {
    SendMovementCommand(sock, 0.0, 3.14); // Approx. 180 degrees in radians
}

// Function to set terminal to raw mode
void SetTerminalRawMode(termios &originalTermios) {
    termios raw = originalTermios;
    raw.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// Function to restore terminal settings
void RestoreTerminalMode(const termios &originalTermios) {
    tcsetattr(STDIN_FILENO, TCSANOW, &originalTermios);
}

// Function to handle robot movement commands interactively
void CommanderHandler(int sock) {
    termios originalTermios;
    tcgetattr(STDIN_FILENO, &originalTermios); // Get current terminal settings

    SetTerminalRawMode(originalTermios); // Set terminal to raw mode

    std::cout << "[INFO] Use arrow keys to move the robot. Press 's' to stop, 'l' for 90° left, 'r' for 90° right, 't' for 180° turn, and 'q' to quit." << std::endl;

    char c;
    while (true) {
        c = getchar();

        switch (c) {
            case 'A': // Up arrow
                MoveForward(sock);
                break;
            case 'B': // Down arrow
                MoveBackward(sock);
                break;
            case 'C': // Right arrow
                MoveRight(sock);
                break;
            case 'D': // Left arrow
                MoveLeft(sock);
                break;
            case 's': // Stop
                StopRobot(sock);
                break;
            case 'l': // Rotate left 90 degrees
                RotateLeft90(sock);
                break;
            case 'r': // Rotate right 90 degrees
                RotateRight90(sock);
                break;
            case 't': // Rotate 180 degrees
                Rotate180(sock);
                break;
            case 'q': // Quit
                StopRobot(sock);
                RestoreTerminalMode(originalTermios);
                std::cout << "[INFO] Exiting Commander." << std::endl;
                return;
            default:
                continue;
        }
    }

    RestoreTerminalMode(originalTermios); // Restore terminal settings before exiting
}
