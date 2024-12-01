#include "TCPServerLaunch.h"

void TCPServerLaunch(unsigned short port) {
    // Create the server socket
    int serverSock = CreateTCPServerSocket(port);
    std::cout << "Server is listening on port " << port << std::endl;

    for (;;) {
        // Accept a new connection
        int clientSock = AcceptTCPConnection(serverSock);

        // Handle the client connection
        HandleTCPClient(clientSock);
    }

    close(serverSock); // Close the server socket (this is never reached)
}
