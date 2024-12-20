#ifndef CONNECT_TO_SERVER_H
#define CONNECT_TO_SERVER_H

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

/**
 * Connects to a server on a specific port.
 * 
 * @param serverIP The IP address of the server to connect to.
 * @param port The port number to connect to.
 * @return The socket descriptor for the connection.
 * 
 * @throws Exits the program if the connection fails.
 */
int ConnectToServer(const char *serverIP, unsigned short port);

#endif // CONNECT_TO_SERVER_H
