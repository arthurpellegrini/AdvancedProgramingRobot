#ifndef CONNECT_TO_SERVER_H
#define CONNECT_TO_SERVER_H

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

/**
 * @file ConnectToServer.h
 * @brief Provides a function to establish a connection to a server.
 * 
 * This header defines the function required to connect to a server
 * using a specified IP address and port number.
 */

/**
 * @brief Connects to a server on a specific port.
 * 
 * Establishes a TCP connection to a server using the given IP address
 * and port number. This function is designed for use in scenarios
 * requiring reliable data transmission.
 * 
 * @param serverIP The IP address of the server to connect to.
 * @param port The port number to connect to.
 * @return The socket descriptor for the connection if successful.
 * 
 * @throws Exits the program if the connection fails, outputting an error message to stderr.
 * 
 * @note Ensure that the server is running and accessible before invoking this function.
 */
int ConnectToServer(const char *serverIP, unsigned short port);

#endif // CONNECT_TO_SERVER_H
