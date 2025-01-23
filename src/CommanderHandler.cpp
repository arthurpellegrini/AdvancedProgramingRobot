#include "CommanderHandler.h"
#include <cmath>

#define BUFFER_SIZE 1024 ///< Buffer size for TCP communication

/**
 * @file CommanderHandler.cpp
 * 
 * @brief Connects to the Commander server.
 * 
 * Establishes a TCP connection to the Commander server using the specified
 * IP address and port.
 * 
 * @param serverIP The IP address of the server.
 * @param port The port number to connect to.
 * @return The socket descriptor for the connection.
 * 
 * @throws Exits the program if the connection fails.
 */
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

/**
 * @brief Sends a movement command to the robot.
 * 
 * Constructs a command string with the specified linear and angular velocities
 * and sends it over the specified socket.
 * 
 * @param sock The socket descriptor for the connection.
 * @param linear The linear velocity (in meters per second).
 * @param angular The angular velocity (in radians per second).
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
    } else {
        std::cout << "[DEBUG] Sent command: " << command << std::endl;
    }
}

// Movement commands
/**
 * @brief Moves the robot forward.
 * @param sock The socket descriptor for the connection.
 */
void MoveForward(int sock) {
    SendMovementCommand(sock, 0.1, 0.0);
}

/**
 * @brief Moves the robot backward.
 * @param sock The socket descriptor for the connection.
 */
void MoveBackward(int sock) {
    SendMovementCommand(sock, -0.1, 0.0);
}

/**
 * @brief Rotates the robot to the left.
 * @param sock The socket descriptor for the connection.
 */
void MoveLeft(int sock) {
    SendMovementCommand(sock, 0.0, 0.5);
}

/**
 * @brief Rotates the robot to the right.
 * @param sock The socket descriptor for the connection.
 */
void MoveRight(int sock) {
    SendMovementCommand(sock, 0.0, -0.5);
}

void LinearControl(int sock, SharedData* shared, sem_t* odometrySemaphore) {
    float targetX, targetY;

    // Get target pose
    std::cout << "Enter target X: ";
    std::cin >> targetX;
    std::cout << "Enter target Y: ";
    std::cin >> targetY;

    // Hyperparameters
    const float kRho = 0.5f;     // Proportional gain for linear velocity
    const float kAlpha = 1.0f;   // Proportional gain for angular velocity (angle to goal)
    const float kBeta = -0.5f;   // Proportional gain for angular velocity (angle to goal)

    while (true) {
        // Read current odometry data
        sem_wait(odometrySemaphore);
        std::string jsonString(shared->odometryData);
        sem_post(odometrySemaphore);

        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errs;
        std::istringstream istr(jsonString);

        float currentX = 0.0f, currentY = 0.0f, currentTheta = 0.0f;
        if (Json::parseFromStream(builder, istr, &root, &errs)) {
            if (root["pose"].isMember("pose")) {
                auto pose = root["pose"]["pose"];
                if (pose["position"].isMember("x") && pose["position"].isMember("y")) {
                    currentX = pose["position"]["x"].asFloat();
                    currentY = pose["position"]["y"].asFloat();
                }
                if (pose["orientation"].isMember("w") && pose["orientation"].isMember("z")) {
                    float w = pose["orientation"]["w"].asFloat();
                    float z = pose["orientation"]["z"].asFloat();
                    currentTheta = atan2(2.0f * w * z, 1.0f - 2.0f * (z * z));
                }
            }
        }

        // Calculate pose difference
        float deltaX = targetX - currentX;
        float deltaY = targetY - currentY;

        // Calculate polar coordinates
        float rho = sqrt(deltaX * deltaX + deltaY * deltaY);               // Distance to goal
        float alpha = -currentTheta + atan2(deltaY, deltaX);               // Angle to goal
        float beta = -currentTheta - alpha;                                // Angle to goal

        // Check if the goal has been reached
        if (rho < 0.1f) {
            StopRobot(sock);
            std::cout << "[INFO] Goal pose reached." << std::endl;
            break;
        }

        // Compute motion commands
        float v = kRho * rho;                              // Linear velocity
        float omega = kAlpha * alpha + kBeta * beta;       // Angular velocity

        // Send commands to the robot
        SendMovementCommand(sock, v, omega);

        usleep(100000); // 100ms delay
    }

    // Stop the robot at the end
    StopRobot(sock);
}



/**
 * @brief Stops the robot.
 * @param sock The socket descriptor for the connection.
 */
void StopRobot(int sock) {
    SendMovementCommand(sock, 0.0, 0.0);
}

/**
 * @brief Sets the terminal to raw mode.
 * 
 * Configures the terminal for raw input mode, disabling canonical mode
 * and echo, allowing direct keypress capture.
 * 
 * @param originalTermios A reference to store the original terminal settings.
 */
void SetTerminalRawMode(termios &originalTermios) {
    termios raw = originalTermios;
    raw.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

/**
 * @brief Restores the terminal settings.
 * 
 * Resets the terminal to its original state using the provided termios structure.
 * 
 * @param originalTermios The original terminal settings to restore.
 */
void RestoreTerminalMode(const termios &originalTermios) {
    tcsetattr(STDIN_FILENO, TCSANOW, &originalTermios);
}

/**
 * @brief Handles interactive robot movement commands.
 * 
 * Allows the user to control the robot via keyboard input. Supported commands:
 * - Arrow keys: Move forward, backward, left, or right.
 * - 's': Stop the robot.
 * - 'l': Rotate 90 degrees left.
 * - 'r': Rotate 90 degrees right.
 * - 't': Rotate 180 degrees.
 * - 'q': Quit the Commander.
 * 
 * @param sock The socket descriptor for the connection.
 */
void CommanderHandler(int sock, SharedData* shared, sem_t* odometrySemaphore) {
    termios originalTermios;
    tcgetattr(STDIN_FILENO, &originalTermios); // Get current terminal settings

    SetTerminalRawMode(originalTermios); // Set terminal to raw mode

    std::cout << "[INFO] Use arrow keys to move the robot. Press 's' to stop, 'l' for linear_control, and 'q' to quit." << std::endl;

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
            case 'l': // Linear Control
                LinearControl(sock, shared, odometrySemaphore);
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

