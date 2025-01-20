#ifndef LISTEN_ON_TCP_PORT_H
#define LISTEN_ON_TCP_PORT_H

#include <iomanip> // For std::put_time

/**
 * @file ListenOnTCPPort.h
 * @brief Provides functionality to listen on a TCP port and process incoming connections.
 * 
 * This header defines the function for setting up a TCP server socket to accept 
 * and handle incoming client connections. It displays received data for debugging or monitoring purposes.
 */

/**
 * @brief Listens on a specified TCP port and processes incoming connections.
 * 
 * This function initializes a server socket, binds it to the specified port, and listens
 * for incoming TCP connections. When a client connects, the function receives data from the
 * client and displays it on the console.
 * 
 * @param port The port number to listen on.
 * @return Returns 0 on successful execution or a non-zero value if an error occurs.
 * 
 * @note Ensure that the specified port is available and not blocked by a firewall.
 * @note This function is blocking and will run indefinitely unless an error occurs or terminated manually.
 * 
 * @details The function is designed for debugging and demonstration purposes, providing a
 * simple way to test TCP client-server communication. It can be extended for more complex
 * use cases like processing received data or handling multiple clients.
 */
int listenOnTCPPort(int port);

#endif // LISTEN_ON_TCP_PORT_H
