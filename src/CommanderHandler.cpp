/**
 * @file CommanderHandler.cpp
 * @brief Implementation of the Commander Handler for robotic control.
 * 
 * This file contains the implementation of the Commander Handler, which includes
 * functions for connecting to a server, sending movement commands, and handling
 * interactive robot control. It supports features such as linear control and
 * keyboard input for real-time robot movement.
 */

#include "CommanderHandler.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <jsoncpp/json/json.h>
#include <termios.h>

#define BUFFER_SIZE 1024

/**
 * @brief Connects to the commander server on a specified IP and port.
 * 
 * This function creates a TCP socket and attempts to connect to the given server.
 * It retries the connection until successful and provides debug information during
 * the process.
 * 
 * @param serverIP The IP address of the commander server.
 * @param port The port number to connect to.
 * @return The socket descriptor for the established connection.
 * 
 * @throws Terminates the program if socket creation or connection fails.
 */
int ConnectToCommanderServer(const char *serverIP, int port) {
    int sock;
    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);

    if (inet_pton(AF_INET, serverIP, &serverAddr.sin_addr) <= 0) {
        perror("Invalid address");
        exit(1);
    }

    while (true) {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            perror("Socket creation failed");
            sleep(1); // Retry after delay
            continue;
        }

        if (connect(sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
            perror("Connection failed, retrying...");
            close(sock);
            sleep(5); // Retry after delay
            continue;
        }

        std::cout << "[INFO] Connected to " << serverIP << ":" << port << std::endl;
        break;
    }

    return sock;
}

/**
 * @brief Sends a movement command to the robot.
 * 
 * Constructs a JSON command string with the specified linear and angular velocities
 * and sends it to the connected server.
 * 
 * @param sock The socket descriptor for the commander server connection.
 * @param linear The linear velocity of the robot (m/s).
 * @param angular The angular velocity of the robot (rad/s).
 */
void SendMovementCommand(int sock, float linear, float angular) {
    std::ostringstream oss;
    oss << "---START---{";
    oss << "\"linear\": " << linear << ", ";
    oss << "\"angular\": " << angular;
    oss << "}___END___";
    std::string command = oss.str();

    if (send(sock, command.c_str(), command.length(), 0) < 0) {
        perror("[ERROR] Failed to send command");
    }
}

/** @brief Moves the robot forward. */
void MoveForward(int sock) { SendMovementCommand(sock, 0.1, 0.0); }

/** @brief Moves the robot backward. */
void MoveBackward(int sock) { SendMovementCommand(sock, -0.1, 0.0); }

/** @brief Rotates the robot to the left. */
void MoveLeft(int sock) { SendMovementCommand(sock, 0.0, 0.5); }

/** @brief Rotates the robot to the right. */
void MoveRight(int sock) { SendMovementCommand(sock, 0.0, -0.5); }

/** @brief Stops the robot. */
void StopRobot(int sock) { SendMovementCommand(sock, 0.0, 0.0); }

/**
 * @brief Normalizes an angle to the range [-π, π].
 * 
 * @param angle The angle to normalize (in radians).
 * @return The normalized angle in the range [-π, π].
 */
float NormalizeAngle(float angle) {
    angle = fmod(angle + M_PI, 2.0f * M_PI);
    if (angle < 0) angle += 2.0f * M_PI;
    return angle - M_PI;
}

/** Flag to control the termination of linear control. */
extern bool stopControl;

/**
 * @brief Handles linear control of the robot using odometry data.
 * 
 * This function calculates the required velocities to move the robot to
 * a specified target position using proportional control.
 * 
 * @param sock The socket descriptor for the commander server connection.
 * @param shared Shared memory containing odometry data.
 * @param odometrySemaphore Semaphore to manage access to shared memory.
 */
void LinearControl(int sock, SharedData* shared, sem_t* odometrySemaphore) {
    float targetX, targetY;
    std::cout << "Enter target X: "; std::cin >> targetX;
    std::cout << "Enter target Y: "; std::cin >> targetY;

    const float kRho = 0.5f, kAlpha = 1.0f, kBeta = -0.5f;
    
    while (!stopControl) {
        if (sem_trywait(odometrySemaphore) != 0) {
            std::cerr << "[WARNING] Semaphore unavailable, skipping cycle!" << std::endl;
            continue;
        }

        std::string jsonString(shared->odometryData);
        sem_post(odometrySemaphore);
        if (jsonString.empty()) {
            std::cerr << "[ERROR] Empty odometry data!" << std::endl;
            continue;
        }

        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errs;
        std::istringstream istr(jsonString);
        
        if (!Json::parseFromStream(builder, istr, &root, &errs)) {
            std::cerr << "[ERROR] JSON parse failed: " << errs << std::endl;
            continue;
        }
        
        float currentX = root["pose"]["pose"]["position"]["x"].asFloat();
        float currentY = root["pose"]["pose"]["position"]["y"].asFloat();
        float w = root["pose"]["pose"]["orientation"]["w"].asFloat();
        float z = root["pose"]["pose"]["orientation"]["z"].asFloat();
        float currentTheta = atan2(2.0f * w * z, 1.0f - 2.0f * (z * z));
        
        float deltaX = targetX - currentX;
        float deltaY = targetY - currentY;
        float rho = sqrt(deltaX * deltaX + deltaY * deltaY);
        float goalTheta = atan2(deltaY, deltaX);
        float alpha = NormalizeAngle(goalTheta - currentTheta);
        float beta = NormalizeAngle(goalTheta - currentTheta - alpha);

        if (rho < 0.05f) { StopRobot(sock); break; }

        SendMovementCommand(sock, kRho * rho, kAlpha * alpha + kBeta * beta);
        usleep(100000);
    }
    StopRobot(sock);
}

/**
 * @brief Sets the terminal to raw mode.
 * 
 * @param originalTermios The original terminal settings to restore later.
 */
void SetTerminalRawMode(termios &originalTermios) {
    termios raw = originalTermios;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

/**
 * @brief Restores the terminal to its original mode.
 * 
 * @param originalTermios The original terminal settings to restore.
 */
void RestoreTerminalMode(const termios &originalTermios) {
    tcsetattr(STDIN_FILENO, TCSANOW, &originalTermios);
}

/**
 * @brief Handles interactive robot control via keyboard input.
 * 
 * The function supports movement commands (arrow keys), stopping ('s'),
 * linear control ('l'), and quitting ('q').
 * 
 * @param sock The socket descriptor for the commander server connection.
 * @param shared Shared memory containing odometry data.
 * @param odometrySemaphore Semaphore to manage access to shared memory.
 */
void CommanderHandler(int sock, SharedData* shared, sem_t* odometrySemaphore) {
    termios originalTermios;
    tcgetattr(STDIN_FILENO, &originalTermios);
    SetTerminalRawMode(originalTermios);

    std::cout << "[INFO] Use arrow keys to move, 's' to stop, 'l' for linear control, 'q' to quit." << std::endl;

    while (true) {
        char buf[3] = {0};
        if (read(STDIN_FILENO, buf, 3) < 0) continue;

        if (buf[0] == '\033' && buf[1] == '[') {
            switch (buf[2]) {
                case 'A': MoveForward(sock); break;
                case 'B': MoveBackward(sock); break;
                case 'C': MoveRight(sock); break;
                case 'D': MoveLeft(sock); break;
            }
        } else {
            switch (buf[0]) {
                case 's': StopRobot(sock); break;
                case 'l': LinearControl(sock, shared, odometrySemaphore); break;
                case 'q': StopRobot(sock); RestoreTerminalMode(originalTermios); return;
            }
        }
    }
    RestoreTerminalMode(originalTermios);
}
