#ifndef COMMANDER_HANDLER_H
#define COMMANDER_HANDLER_H

#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <string>
#include <sstream>
#include <jsoncpp/json/json.h>
#include "SharedMemory.h"
#include <semaphore.h>

#define BUFFER_SIZE 1024

/**
 * @file CommanderHandler.h
 * 
 * @brief Connects to the Commander server.
 * 
 * Establishes a connection to the server handling Commander commands
 * for controlling the robot's movement.
 * 
 * @param serverIP The IP address of the server.
 * @param port The port number to connect to.
 * @return Returns the socket descriptor if successful, or -1 on failure.
 */
int ConnectToCommanderServer(const char *serverIP, int port);

/**
 * @brief Sends a movement command to the robot.
 * 
 * Constructs and sends a movement command to control the robot's
 * linear and angular velocities.
 * 
 * @param sock The socket descriptor for the Commander server connection.
 * @param linear The desired linear velocity.
 * @param angular The desired angular velocity.
 */
void SendMovementCommand(int sock, float linear, float angular);

/**
 * @brief Commands the robot to move forward.
 * 
 * Sends a predefined command to move the robot forward at a fixed speed.
 * 
 * @param sock The socket descriptor for the Commander server connection.
 */
void MoveForward(int sock);

/**
 * @brief Commands the robot to move backward.
 * 
 * Sends a predefined command to move the robot backward at a fixed speed.
 * 
 * @param sock The socket descriptor for the Commander server connection.
 */
void MoveBackward(int sock);

/**
 * @brief Commands the robot to turn left.
 * 
 * Sends a predefined command to turn the robot left.
 * 
 * @param sock The socket descriptor for the Commander server connection.
 */
void MoveLeft(int sock);

/**
 * @brief Commands the robot to turn right.
 * 
 * Sends a predefined command to turn the robot right.
 * 
 * @param sock The socket descriptor for the Commander server connection.
 */
void MoveRight(int sock);

/**
 * @brief Commands the robot to stop.
 * 
 * Sends a predefined command to halt the robot's movement immediately.
 * 
 * @param sock The socket descriptor for the Commander server connection.
 */
void StopRobot(int sock);

/**
 * @brief Sets the terminal to raw mode.
 * 
 * Configures the terminal for raw input mode, disabling line buffering
 * and other input processing. Useful for capturing keystrokes directly.
 * 
 * @param originalTermios A reference to a termios structure to store the original settings.
 */
void SetTerminalRawMode(termios &originalTermios);

/**
 * @brief Restores the terminal to its original mode.
 * 
 * Resets the terminal settings to the state stored in the given termios structure.
 * 
 * @param originalTermios A reference to the termios structure containing the original settings.
 */
void RestoreTerminalMode(const termios &originalTermios);

/**
 * @brief Handles robot movement commands interactively.
 * 
 * Allows the user to control the robot via keyboard input.
 * Interprets key presses and sends corresponding movement commands
 * to the robot through the Commander server connection.
 * 
 * @param sock The socket descriptor for the Commander server connection.
 */
void CommanderHandler(int sock, SharedData* shared, sem_t* odometrySemaphore);

#endif // COMMANDER_HANDLER_H
