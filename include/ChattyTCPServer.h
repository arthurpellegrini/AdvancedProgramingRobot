#ifndef CHATTY_TCP_SERVER_H
#define CHATTY_TCP_SERVER_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstdlib>

// Function to start the chatty TCP server
void StartChattyTCPServer(unsigned short port);

#endif // CHATTY_TCP_SERVER_H
