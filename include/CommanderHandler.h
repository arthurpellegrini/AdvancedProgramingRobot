#ifndef COMMANDER_HANDLER_H
#define COMMANDER_HANDLER_H

#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <string>
#include <sstream>

#define BUFFER_SIZE 1024

// Function to connect to the Commander server
int ConnectToCommanderServer(const char *serverIP, int port);

// Function to send movement commands
void SendMovementCommand(int sock, float linear, float angular);

// Movement functions
void MoveForward(int sock);
void MoveBackward(int sock);
void MoveLeft(int sock);
void MoveRight(int sock);
void StopRobot(int sock);

// Terminal mode functions
void SetTerminalRawMode(termios &originalTermios);
void RestoreTerminalMode(const termios &originalTermios);

// Function to handle robot movement commands interactively
void CommanderHandler(int sock);

#endif // COMMANDER_HANDLER_H
