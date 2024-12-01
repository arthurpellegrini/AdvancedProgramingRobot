#ifndef TCP_SERVER_LAUNCH_H
#define TCP_SERVER_LAUNCH_H

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include "CreateTCPServerSocket.h"
#include "AcceptTCPConnection.h"
#include "HandleTCPClient.h"
#include "DieWithError.h"

// Function to launch the TCP server
void TCPServerLaunch(unsigned short port);

#endif // TCP_SERVER_LAUNCH_H
