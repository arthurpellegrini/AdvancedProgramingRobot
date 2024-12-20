#ifndef LISTEN_ON_TCP_PORT_H
#define LISTEN_ON_TCP_PORT_H

#include <iomanip> // For std::put_time

/**
 * Listens on a specified TCP port and processes incoming connections.
 *
 * This function sets up a TCP server socket, accepts incoming connections,
 * and displays data received from clients on the console.
 *
 * @param port The port number to listen on.
 * @return 0 on success, or a non-zero value on error.
 */
int listenOnTCPPort(int port);

#endif // LISTEN_ON_TCP_PORT_H
