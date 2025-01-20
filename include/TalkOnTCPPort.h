#ifndef TALK_ON_TRCP_PORT_H
#define TALK_ON_TRCP_PORT_H

#include <string>

/**
 * @file TalkOnTCPPort.h
 * @brief Provides functionality to send movement commands to the TurtleBot via a TCP socket.
 * 
 * This header defines a function for communicating movement commands to the TurtleBot
 * over a TCP connection. The function allows control of linear and angular velocities.
 */

/**
 * @brief Sends a movement command to the TurtleBot via a TCP socket.
 * 
 * This function constructs a movement command with the specified linear and angular velocities
 * and sends it to the TurtleBot through the provided socket.
 * 
 * @param socket The socket descriptor for the TCP connection.
 * @param linear_x The linear velocity for the TurtleBot (in meters per second).
 * @param angular_z The angular velocity for the TurtleBot (in radians per second).
 * 
 * @note Ensure that the socket is connected to the TurtleBot's command server before calling this function.
 * 
 * @details This function sends the command in a predefined format that the TurtleBot's command server can interpret.
 * The exact format of the command should match the protocol expected by the TurtleBot.
 */
void startTalkOnTCPPort(int socket, float linear_x, float angular_z);

#endif // TALK_ON_TRCP_PORT_H
